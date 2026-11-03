#pragma once
#include <array>
#include <cerrno>
#include <cstdint>
#include <cstring>
#include <string>
#include <type_traits>
#include <sys/ipc.h>
#include <sys/shm.h>
#include <signal.h>
#include <unistd.h>

namespace kuavo_reference_shm {
// Linux SysV snapshot transport. Two independent single-writer regions are
// required: producer->consumer and execution-owner->producer. Payload words
// are atomic too: a seqlock alone would not make concurrent memcpy race-free.
template<class Payload, uint32_t Channel>
class Snapshot {
  static_assert(std::is_trivially_copyable<Payload>::value, "fixed ABI only");
  static_assert(__atomic_always_lock_free(8, nullptr), "64-bit atomic required");
  static constexpr uint64_t Magic=0x5742494b53484d31ULL;
  static constexpr std::size_t Words=(sizeof(Payload)+7)/8;
  struct alignas(64) Region {
    uint64_t magic, abi;
    uint32_t version, bytes, channel, reserved;
    alignas(8) uint64_t sequence;
    uint32_t writer_pid, padding;
    alignas(64) uint64_t words[Words];
  };
 public:
  enum class Role { Reader, Writer };
  Snapshot()=default;
  Snapshot(const Snapshot&)=delete;
  Snapshot& operator=(const Snapshot&)=delete;
  ~Snapshot(){close();}
  bool initialize(int key,Role role) {
    if(region_)return false;
    if(key<=0){error_="key must be positive";return false;}
    role_=role; bool created=false;
    int id=shmget(key,sizeof(Region),IPC_CREAT|IPC_EXCL|0666);
    if(id>=0)created=true;
    else if(errno==EEXIST)id=shmget(key,1,0666);
    if(id<0){error_=std::strerror(errno);return false;}
    shmid_ds info{};
    if(shmctl(id,IPC_STAT,&info)<0 || info.shm_segsz!=sizeof(Region)){
      error_="shared region ABI size mismatch";return false;
    }
    void* address=shmat(id,nullptr,0);
    if(address==reinterpret_cast<void*>(-1)){error_=std::strerror(errno);return false;}
    region_=static_cast<Region*>(address);id_=id;
    if(created){
      // Only IPC_EXCL creator initializes. Readers never erase an existing
      // writer's data; release magic makes completed startup visible.
      std::memset(region_,0,sizeof(Region));
      region_->abi=Payload::ABI_TAG;region_->version=1;region_->bytes=sizeof(Region);region_->channel=Channel;
      __atomic_store_n(&region_->magic,Magic,__ATOMIC_RELEASE);
    }
    if(__atomic_load_n(&region_->magic,__ATOMIC_ACQUIRE)!=Magic ||
       region_->abi!=Payload::ABI_TAG || region_->version!=1 || region_->bytes!=sizeof(Region) ||
       region_->channel!=Channel){error_="shared region ABI/channel mismatch";close();return false;}
    if(role==Role::Writer){
      const uint32_t self=static_cast<uint32_t>(getpid());
      uint32_t previous=__atomic_load_n(&region_->writer_pid,__ATOMIC_ACQUIRE);
      if(previous && (kill(previous,0)==0 || errno!=ESRCH)){
        error_="another writer owns this region";close();return false;
      }
      if(!__atomic_compare_exchange_n(&region_->writer_pid,&previous,self,false,
                                     __ATOMIC_ACQ_REL,__ATOMIC_ACQUIRE)){
        error_="writer claim raced";close();return false;
      }
      owns_writer_=true;
      Payload empty{}; // invalidate predecessor before any new publication
      if(!write(empty)){error_="cannot invalidate predecessor";close();return false;}
    }
    return true;
  }
  void close(){
    if(!region_)return;
    if(owns_writer_){
      uint32_t self=static_cast<uint32_t>(getpid());
      __atomic_compare_exchange_n(&region_->writer_pid,&self,0,false,
                                   __ATOMIC_RELEASE,__ATOMIC_RELAXED);
    }
    shmdt(region_);region_=nullptr;owns_writer_=false;id_=-1;last_read_=0;
  }
  bool write(const Payload& value){
    if(!region_ || role_!=Role::Writer || !owns_writer_ ||
       __atomic_load_n(&region_->writer_pid,__ATOMIC_ACQUIRE)!=uint32_t(getpid()))return false;
    uint64_t base=__atomic_load_n(&region_->sequence,__ATOMIC_RELAXED);
    base+=base&1ULL;
    if(base>UINT64_MAX-2)return false;
    std::array<uint64_t,Words> words{};std::memcpy(words.data(),&value,sizeof(value));
    // A full fence orders odd publication before the subsequent payload stores.
    __atomic_store_n(&region_->sequence,base+1,__ATOMIC_SEQ_CST);
    for(std::size_t i=0;i<Words;++i)__atomic_store_n(&region_->words[i],words[i],__ATOMIC_RELAXED);
    __atomic_store_n(&region_->sequence,base+2,__ATOMIC_RELEASE);
    last_write_=base+2;return true;
  }
  bool readIfUpdated(Payload& value){
    if(!region_ || role_!=Role::Reader)return false;
    for(int attempt=0;attempt<3;++attempt){
      const uint32_t writer=__atomic_load_n(&region_->writer_pid,__ATOMIC_ACQUIRE);
      if(!writer)return false;
      const uint64_t first=__atomic_load_n(&region_->sequence,__ATOMIC_ACQUIRE);
      if(!first || first==last_read_)return false;
      if(first&1ULL){++busy_reads_;continue;}
      std::array<uint64_t,Words> words{};
      for(std::size_t i=0;i<Words;++i)words[i]=__atomic_load_n(&region_->words[i],__ATOMIC_RELAXED);
      __atomic_thread_fence(__ATOMIC_SEQ_CST);
      const uint64_t second=__atomic_load_n(&region_->sequence,__ATOMIC_ACQUIRE);
      if(first!=second || writer!=__atomic_load_n(&region_->writer_pid,__ATOMIC_ACQUIRE)){
        ++busy_reads_;continue;
      }
      std::memcpy(&value,words.data(),sizeof(value));last_read_=second;return true;
    }
    return false; // bounded read: never spin/wait in a control cycle
  }
  const std::string& error()const{return error_;}
  uint64_t lastRead()const{return last_read_;}
  uint64_t lastWrite()const{return last_write_;}
  uint64_t busyReads()const{return busy_reads_;}
  // Only isolated tests explicitly remove their own keys. Runtime never RMIDs
  // another process's segment, including at restart/timeout.
  bool removeForTest(){return id_>=0 && shmctl(id_,IPC_RMID,nullptr)==0;}
 private:
  Region* region_{nullptr};int id_{-1};Role role_{Role::Reader};
  bool owns_writer_{false};uint64_t last_read_{0},last_write_{0},busy_reads_{0};
  std::string error_;
};
} // namespace kuavo_reference_shm
