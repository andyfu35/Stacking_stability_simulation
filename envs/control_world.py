import pybullet as p
import pybullet_data
import time
import numpy as np
from math import degrees
from envs.object_create import create_box, create_container


class ControlWorld:
    def __init__(self, world_angle=0, box_info=(0.213, 0.255, 0.113, 5), strategy_parameters=(0.01, 0.5)):
        self.world_angle = np.radians(world_angle)
        self.box_info = box_info
        self.drop_height = 0.01
        self.strategy_parameters = strategy_parameters
        self.gravity_magnitude = 9.81
        self.gravity_x = self.gravity_magnitude * np.sin(self.world_angle)
        self.gravity_z = self.gravity_magnitude * np.cos(self.world_angle)
        self.container_size = (3.0, 2.35, 2.36)
        self.physics_client = p.connect(p.GUI)
        self.max_col = int((self.container_size[2] - self.drop_height) / self.box_info[2])
        self.max_row = int((self.container_size[1] - 0.005) / self.box_info[1])
        self.sharking_degree = 5
        self.boxes_position = []
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.configureDebugVisualizer(p.COV_ENABLE_SHADOWS, 0)
        p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)
        p.setGravity(self.gravity_x, 0, -self.gravity_z)
        p.setPhysicsEngineParameter(
            fixedTimeStep=1 / 100,
            numSolverIterations=200,
            contactSlop=0.0001,
            useSplitImpulse=True,
            splitImpulsePenetrationThreshold=-0.02
        )

    def setup_environment(self):
        create_container(self.container_size)

    def update_strategy_parameters(self, strategy_parameters):
        self.strategy_parameters = strategy_parameters

    def place_strategy(self, current_col=0):
        return self.strategy_parameters[0] * (current_col / self.max_col) ** self.strategy_parameters[1]

    def generate_box_positions(self):
        for x_ in range(1):
            for col in range(self.max_col):
                current_depth = self.place_strategy(col)
                box_x = self.box_info[0] / 2 + self.strategy_parameters[0] + (
                            self.box_info[0] + 0.001) * x_ - current_depth
                box_z = self.box_info[2] / 2 + self.box_info[2] * col + self.drop_height
                for row in range(self.max_row):
                    box_y = self.container_size[1] / 2 - 0.001 - self.box_info[1] / 2 - (self.box_info[1] + 0.001) * row
                    self.boxes_position.append((box_x, box_y, box_z))
        return self.boxes_position

    def place_box(self):
        box_pos_list = self.generate_box_positions()
        for box in box_pos_list:
            create_box(self.box_info, box)
            time.sleep(0.1)

    @staticmethod
    def wait_until_stable(threshold=0.002, check_interval=0.5, max_wait=100, max_pitch=45):
        """
        等待箱子穩定，並檢測是否有繞 Y 軸的旋轉角度過大。
        :param threshold: 判定穩定的速度和角速度閾值。
        :param check_interval: 每次檢查的時間間隔（秒）。
        :param max_wait: 最大等待時間（秒）。
        :param max_pitch: 最大允許的繞 Y 軸旋轉角度（度）。
        """
        start_time = time.time()

        while time.time() - start_time < max_wait:
            all_stable = True

            for body_id in range(p.getNumBodies()):
                mass, _, _ = p.getDynamicsInfo(body_id, -1)[:3]
                if mass <= 0:
                    continue

                linear_velocity, angular_velocity = p.getBaseVelocity(body_id)
                position, orientation = p.getBasePositionAndOrientation(body_id)
                # print(linear_velocity,"  ",angular_velocity)
                # 獲取繞 Y 軸的旋轉角度
                _, pitch, _ = p.getEulerFromQuaternion(orientation)
                pitch_angle = degrees(pitch)

                # 判斷繞 Y 軸的旋轉角度是否超過允許範圍
                if abs(pitch_angle) > max_pitch:
                    print(f"Box {body_id} failed due to excessive Y-axis rotation: Pitch = {pitch_angle}°")
                    return False

                # 判斷速度是否穩定
                if any(abs(v) > threshold for v in linear_velocity) or any(
                        abs(w) > threshold for w in angular_velocity):
                    all_stable = False
                    break

            if all_stable:
                print("All boxes are stable.")
                return True

            time.sleep(check_interval)

        print("Timeout: Boxes did not stabilize.")
        return False

    def simulate_sharking(self):
        shaking_amplitude = np.radians(self.sharking_degree)
        sharking_amplitude_ = self.gravity_magnitude * np.sin(shaking_amplitude)
        sharking = [(-sharking_amplitude_, 0, -self.gravity_z),
                    (sharking_amplitude_, 0, -self.gravity_z)]

        for times in range(10):
            for shark in sharking:
                p.setGravity(shark[0], shark[1], shark[2])
                time.sleep(0.3)
        p.setGravity(self.gravity_x, 0, -self.gravity_z)

    def reset_world(self):
        p.resetSimulation()
        self.setup_environment()
        p.setGravity(self.gravity_x, 0, -self.gravity_z)
        self.boxes_position = []

    @staticmethod
    def run_simulation():
        while True:
            p.stepSimulation()
            time.sleep(1 / 1000)

    @staticmethod
    def close_windows():
        p.disconnect()

    def strategy_score(self):
        """
        根據所有箱子的穩定性進行評分。
        """
        if not self.wait_until_stable(threshold=0.01, check_interval=0.5, max_wait=100, max_pitch=45):
            print("Still moving or already failed!")
            return 0

        num_bodies = p.getNumBodies()
        total_pitch = 0
        valid_boxes = 0


        for body_id in range(num_bodies):
            mass, _, _ = p.getDynamicsInfo(body_id, -1)[:3]
            if mass <= 0:
                continue  # 忽略靜態物體

            position, orientation = p.getBasePositionAndOrientation(body_id)
            _, pitch, _ = map(degrees, p.getEulerFromQuaternion(orientation))  # 確保角度為度

            # 檢測是否超過最大允許範圍
            if abs(pitch) > 45 or position[2] < 0.1:
                print(f"Box {body_id} has fallen! Pitch: {pitch}, Height: {position[2]}")
                return -5  # 直接返回最低分

            total_pitch += abs(pitch)  # 累計絕對值角度
            valid_boxes += 1

        if valid_boxes == 0:
            return 0  # 沒有有效箱子，返回 0 分

        # 計算平均繞 Y 軸的旋轉角度
        avg_pitch = total_pitch / valid_boxes
        print(f"Average Pitch: {avg_pitch}")

        # 根據平均傾斜角度計算懲罰
        penalty = (avg_pitch / 0.5) * 2  # 每 0.5 度扣 2 分

        # 計算最終分數
        final_score = 10 - penalty
        final_score = final_score - self.strategy_parameters[0] * 10
        final_score = max(final_score, 0)  # 分數不低於 0

        print(f"Final Score: {final_score}")
        return final_score