import numpy as np
import os, sys

cur_dir = os.path.dirname(os.path.abspath(__file__))

from pydrake.all import (
    MultibodyPlant,
    Parser,
    RigidTransform,
    RollPitchYaw,
)

class FKTool:
    """正运动学计算工具类."""
    def __init__(self, urdf_path=os.path.join(cur_dir, 'drake_urdf/urdf/biped_v3_arm.urdf')):
        self.plant, self.model_instance = FKTool.load_plant_from_urdf(urdf_path)
        self.context = self.plant.CreateDefaultContext()
        # 2. 取出当前（默认）关节位置
        q = self.plant.GetPositions(self.context, self.model_instance).copy()
        # print("默认 q =", q)  # 有兴趣的话可以看一下结构：前面 7 个一般是 floating base

        q[7:] = 0.0
        # 4. 写回 context
        self.plant.SetPositions(self.context, self.model_instance, q)

    def compute(self, q, frame_name, base_frame_name="base_link", verbose=False):
        # base_frame_name = 'zarm_l7_link'
        FKTool.set_joint_positions(self.plant, self.context, self.model_instance, q)

        # 3. 算末端 link "tool_link" 在 world 下的位姿
        X_WTool = FKTool._compute_fk(self.plant, self.context, frame_name=frame_name, base_frame_name=base_frame_name)

        p = X_WTool.translation()
        R = X_WTool.rotation()  # RotationMatrix
        quat = R.ToQuaternion().wxyz()  # Drake 的四元数格式 xyzw
        rpy = RollPitchYaw(R).vector()  # Drake 的 RPY（space-fixed XYZ）

        if verbose:
            print(f'Frame "{frame_name}" in "{base_frame_name}":')
            print("  平移 p =", p)
            print("  旋转 R =", R)
            print("  四元数 quat (xyzw) =", quat)
        return tuple(p), (quat[1], quat[2], quat[3], quat[0])  # 转成 xyzw 顺序返回

    @staticmethod
    def load_plant_from_urdf(urdf_path: str):
        """从 URDF 创建 MultibodyPlant（无 SceneGraph 简化版）."""
        plant = MultibodyPlant(time_step=0.0)  # 连续系统，FK 用不到时间步
        parser = Parser(plant=plant)
        model_instance = parser.AddModelFromFile(urdf_path)
        plant.Finalize()
        return plant, model_instance

    @staticmethod
    def set_joint_positions(plant: MultibodyPlant,
                            context,
                            model_instance,
                            q_vec):
        """
        写入一组关节角到 plant 的 Context 里.

        q_vec: np.array([...])，长度 = 该 model 的 nv 或关节自由度数
        """
        # 取出该 model 对应的关节坐标在 big q 里的索引
        nq = plant.num_positions(model_instance)
        q_vec_base = plant.GetPositions(context, model_instance).copy()
        q_vec_base[7:] = q_vec
        assert len(q_vec_base) == nq, f"q_vec 长度 {len(q_vec_base)} != nq {nq}"

        # 这里只演示「这个 model 是唯一的」的简单情况
        # 多 model 情况下可以用 GetPositions/SetPositions(model_instance)
        plant.SetPositions(context, model_instance, q_vec_base)


    @staticmethod
    def _compute_fk(plant: MultibodyPlant,
                   context,
                   frame_name: str,
                   base_frame_name: str = "world"):
        """
        计算任意 frame 在 base_frame 下的位姿（RigidTransform）.

        frame_name: 末端或其他 link / frame 的名字（如 "tool_link"）
        base_frame_name: 基准坐标系名，默认 "world"
        """
        # 拿 frame
        frame = plant.GetFrameByName(frame_name)
        if base_frame_name == "world":
            base_frame = plant.world_frame()
        else:
            base_frame = plant.GetFrameByName(base_frame_name)

        X_base_frame = plant.CalcRelativeTransform(
            context,
            frame_B=frame,
            frame_A=base_frame,   # 注意 Drake 这里的命名：T_BA
        )
        return X_base_frame


if __name__ == "__main__":
    urdf_path=os.path.join(cur_dir, 'drake_urdf/urdf/biped_v3_arm.urdf')
    fk_tool = FKTool(urdf_path)
    q = np.zeros(14)

    fk_tool.compute(q, frame_name="base_link", base_frame_name="zarm_l1_link")
