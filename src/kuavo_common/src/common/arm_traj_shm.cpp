#include "kuavo_common/common/arm_traj_shm.h"

#include <cstring>
#include <iostream>

#include <errno.h>
#include <sys/ipc.h>
#include <sys/shm.h>

namespace kuavo_common {

namespace {
void clearWriterPayload(ArmTrajShmData* ptr, uint64_t* write_seq) {
  std::memset(ptr, 0, sizeof(ArmTrajShmData));
  ptr->valid = false;
  __atomic_store_n(&ptr->seq, 0, __ATOMIC_RELEASE);
  if (write_seq) {
    *write_seq = 0;
  }
}
}  // namespace

ArmTrajShmManager::ArmTrajShmManager() = default;

ArmTrajShmManager::~ArmTrajShmManager() {
  cleanup();
}

bool ArmTrajShmManager::attachShm() {
  shm_id_ = shmget(SHM_KEY, sizeof(ArmTrajShmData), IPC_CREAT | 0666);
  if (shm_id_ == -1) {
    std::cerr << "[ArmTrajShmManager] shmget failed: " << std::strerror(errno) << std::endl;
    return false;
  }

  void* ptr = shmat(shm_id_, nullptr, 0);
  if (ptr == reinterpret_cast<void*>(-1)) {
    std::cerr << "[ArmTrajShmManager] shmat failed: " << std::strerror(errno) << std::endl;
    shm_ptr_ = nullptr;
    return false;
  }

  shm_ptr_ = static_cast<ArmTrajShmData*>(ptr);
  return true;
}

bool ArmTrajShmManager::initialize(Role role) {
  if (initialized_) {
    return true;
  }

  role_ = role;
  if (!attachShm()) {
    return false;
  }

  // Writer 清零；不用 POSIX named sem（曾出现 0 字节坏文件 → sem_wait SIGBUS）
  if (role == Role::Writer) {
    clearWriterPayload(shm_ptr_, &write_seq_);
  }

  initialized_ = true;
  return true;
}

void ArmTrajShmManager::cleanup() {
  if (shm_ptr_ != nullptr) {
    shmdt(shm_ptr_);
    shm_ptr_ = nullptr;
  }

  shm_id_ = -1;
  initialized_ = false;
  last_read_seq_ = 0;
  write_seq_ = 0;
}

void ArmTrajShmManager::invalidate() {
  if (!initialized_ || role_ != Role::Writer || shm_ptr_ == nullptr) {
    return;
  }
  clearWriterPayload(shm_ptr_, &write_seq_);
}

bool ArmTrajShmManager::writeTrajRad(uint32_t num_joints,
                                     const double* position_rad,
                                     const double* velocity_rad,
                                     const double* effort,
                                     uint64_t stamp_nsec) {
  if (!initialized_ || role_ != Role::Writer || shm_ptr_ == nullptr) {
    return false;
  }
  if (num_joints == 0 || num_joints > static_cast<uint32_t>(ArmTrajShmData::MAX_ARM_JOINTS)) {
    return false;
  }
  if (position_rad == nullptr || velocity_rad == nullptr) {
    return false;
  }

  // seqlock：先置 seq=0 使 Reader 丢弃半写快照，写完再发布新 seq
  __atomic_store_n(&shm_ptr_->seq, 0, __ATOMIC_RELEASE);
  shm_ptr_->stamp_nsec = stamp_nsec;
  shm_ptr_->num_joints = num_joints;
  for (uint32_t i = 0; i < num_joints; ++i) {
    shm_ptr_->position[i] = position_rad[i];
    shm_ptr_->velocity[i] = velocity_rad[i];
    shm_ptr_->effort[i] = (effort != nullptr) ? effort[i] : 0.0;
  }
  shm_ptr_->valid = true;
  ++write_seq_;
  __atomic_store_n(&shm_ptr_->seq, write_seq_, __ATOMIC_RELEASE);
  return true;
}

bool ArmTrajShmManager::readIfUpdated(ArmTrajShmData& out) {
  if (!initialized_ || role_ != Role::Reader || shm_ptr_ == nullptr) {
    return false;
  }

  // Reader 用 seq 双读一致性快照（不依赖 sem）
  for (int attempt = 0; attempt < 3; ++attempt) {
    const uint64_t seq1 = __atomic_load_n(&shm_ptr_->seq, __ATOMIC_ACQUIRE);
    if (!shm_ptr_->valid || seq1 == 0 || seq1 == last_read_seq_) {
      return false;
    }
    ArmTrajShmData snap = *shm_ptr_;
    const uint64_t seq2 = __atomic_load_n(&shm_ptr_->seq, __ATOMIC_ACQUIRE);
    if (seq1 == seq2 && snap.valid && snap.seq == seq1) {
      out = snap;
      last_read_seq_ = seq1;
      return true;
    }
  }
  return false;
}

}  // namespace kuavo_common
