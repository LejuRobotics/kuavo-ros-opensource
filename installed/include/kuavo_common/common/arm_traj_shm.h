#pragma once

#include <cstdint>
#include <semaphore.h>

namespace kuavo_common {

struct ArmTrajShmData {
  static constexpr int MAX_ARM_JOINTS = 14;

  uint64_t seq = 0;
  uint64_t stamp_nsec = 0;
  uint32_t num_joints = 0;
  bool valid = false;
  double position[MAX_ARM_JOINTS];
  double velocity[MAX_ARM_JOINTS];
  double effort[MAX_ARM_JOINTS];
};

class ArmTrajShmManager {
 public:
  enum class Role { Writer, Reader };

  ArmTrajShmManager();
  ~ArmTrajShmManager();

  ArmTrajShmManager(const ArmTrajShmManager&) = delete;
  ArmTrajShmManager& operator=(const ArmTrajShmManager&) = delete;

  bool initialize(Role role);
  void cleanup();
  bool isInitialized() const { return initialized_; }

  bool writeTrajRad(uint32_t num_joints,
                    const double* position_rad,
                    const double* velocity_rad,
                    const double* effort,
                    uint64_t stamp_nsec);

  bool readIfUpdated(ArmTrajShmData& out);

 private:
  bool attachShm();

  ArmTrajShmData* shm_ptr_ = nullptr;
  int shm_id_ = -1;
  sem_t* sem_ = nullptr;
  Role role_ = Role::Reader;
  uint64_t last_read_seq_ = 0;
  uint64_t write_seq_ = 0;
  bool initialized_ = false;

  static constexpr int SHM_KEY = 343434;
  static constexpr const char* SEM_NAME = "/kuavo_arm_traj_shm_sem";
};

}  // namespace kuavo_common
