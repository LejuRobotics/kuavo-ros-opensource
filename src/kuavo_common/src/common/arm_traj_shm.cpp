#include "kuavo_common/common/arm_traj_shm.h"

#include <cstring>
#include <iostream>

#include <errno.h>
#include <fcntl.h>
#include <sys/ipc.h>
#include <sys/shm.h>

namespace kuavo_common {

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
  sem_ = sem_open(SEM_NAME, O_CREAT, 0666, 1);
  if (sem_ == SEM_FAILED) {
    std::cerr << "[ArmTrajShmManager] sem_open failed: " << std::strerror(errno) << std::endl;
    return false;
  }

  if (!attachShm()) {
    sem_close(sem_);
    sem_ = nullptr;
    return false;
  }

  if (role == Role::Writer) {
    sem_wait(sem_);
    std::memset(shm_ptr_, 0, sizeof(ArmTrajShmData));
    shm_ptr_->valid = false;
    sem_post(sem_);
  }

  initialized_ = true;
  return true;
}

void ArmTrajShmManager::cleanup() {
  if (shm_ptr_ != nullptr) {
    shmdt(shm_ptr_);
    shm_ptr_ = nullptr;
  }

  if (sem_ != nullptr && sem_ != SEM_FAILED) {
    sem_close(sem_);
    sem_ = nullptr;
  }

  shm_id_ = -1;
  initialized_ = false;
  last_read_seq_ = 0;
  write_seq_ = 0;
}

bool ArmTrajShmManager::writeTrajRad(uint32_t num_joints,
                                     const double* position_rad,
                                     const double* velocity_rad,
                                     const double* effort,
                                     uint64_t stamp_nsec) {
  if (!initialized_ || role_ != Role::Writer || shm_ptr_ == nullptr || sem_ == SEM_FAILED) {
    return false;
  }
  if (num_joints == 0 || num_joints > static_cast<uint32_t>(ArmTrajShmData::MAX_ARM_JOINTS)) {
    return false;
  }
  if (position_rad == nullptr || velocity_rad == nullptr) {
    return false;
  }

  sem_wait(sem_);
  ++write_seq_;
  shm_ptr_->seq = write_seq_;
  shm_ptr_->stamp_nsec = stamp_nsec;
  shm_ptr_->num_joints = num_joints;
  for (uint32_t i = 0; i < num_joints; ++i) {
    shm_ptr_->position[i] = position_rad[i];
    shm_ptr_->velocity[i] = velocity_rad[i];
    shm_ptr_->effort[i] = (effort != nullptr) ? effort[i] : 0.0;
  }
  shm_ptr_->valid = true;
  sem_post(sem_);
  return true;
}

bool ArmTrajShmManager::readIfUpdated(ArmTrajShmData& out) {
  if (!initialized_ || role_ != Role::Reader || shm_ptr_ == nullptr || sem_ == SEM_FAILED) {
    return false;
  }

  sem_wait(sem_);
  if (!shm_ptr_->valid || shm_ptr_->seq == last_read_seq_) {
    sem_post(sem_);
    return false;
  }

  out = *shm_ptr_;
  last_read_seq_ = shm_ptr_->seq;
  sem_post(sem_);
  return true;
}

}  // namespace kuavo_common
