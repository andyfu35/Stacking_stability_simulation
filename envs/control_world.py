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

    def place_strategy(self, current_col=0):
        return self.strategy_parameters[0] * (current_col / self.max_col) ** self.strategy_parameters[1]

    def generate_box_positions(self):
        for col in range(self.max_col):
            current_depth = self.place_strategy(col)
            box_x = self.box_info[0] / 2 + self.strategy_parameters[0] - current_depth
            box_z = self.box_info[2] / 2 + self.box_info[2] * col + self.drop_height
            for row in range(self.max_row):
                box_y = self.container_size[1] / 2 - 0.001 - self.box_info[1] / 2 - self.box_info[1] * row
                self.boxes_position.append((box_x, box_y, box_z))
        return self.boxes_position

    def place_box(self):
        box_pos_list = self.generate_box_positions()
        n = 0
        for box in box_pos_list:
            if n%9 == 0:
                self.simulate_sharking()
            n += 1
            create_box(self.box_info, box)
            time.sleep(0.5)

    def simulate_sharking(self):
        sharking_amplitude_1 = self.gravity_magnitude * np.sin(self.sharking_degree)
        sharking_amplitude_2 = self.gravity_magnitude * np.cos(self.sharking_degree)
        sharking = [(-sharking_amplitude_2, 0, sharking_amplitude_1),
                    (0, -sharking_amplitude_2, sharking_amplitude_1),
                    (sharking_amplitude_2, 0, sharking_amplitude_1),
                    (0, sharking_amplitude_2, sharking_amplitude_1)]
        print(sharking)

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

    @staticmethod
    def strategy_score():
        num_bodies = p.getNumBodies()
        total_roll = 0
        total_pitch = 0
        valid_boxes = 0

        for body in range(num_bodies):
            body_id = p.getBodyUniqueId(body)
            mass, _, _ = p.getDynamicsInfo(body_id, -1)[:3]
            if mass <= 0:
                continue

            position, orientation = p.getBasePositionAndOrientation(body_id)
            roll, pitch, _ = map(degrees, p.getEulerFromQuaternion(orientation))
            is_fallen = abs(roll) > 45 or abs(pitch) > 45 or position[2] < 0.1
            if is_fallen:
                return -5

            total_roll += abs(roll)
            total_pitch += abs(pitch)
            valid_boxes += 1

        if valid_boxes == 0:
            return 0

        avg_tilt = (total_roll + total_pitch) / (2 * valid_boxes)

        # 計算總分
        score = max(10 - (avg_tilt / 0.5) * 2, 0)  # 每 0.5 度平均傾斜度扣 2 分
        return score


# def main_thread():
#     sim = ControlWorld(world_angle=5)
#     sim.setup_environment()
#
#     # 創建副線程來運行模擬
#     simulation_thread = threading.Thread(target=sim.run_simulation)
#     place_box_thread = threading.Thread(target=sim.place_box)
#
#     simulation_thread.start()
#     place_box_thread.start()
#     place_box_thread.join()
#
#
#     simulate_sharking_thread = threading.Thread(target=sim.simulate_sharking)
#     simulate_sharking_thread.start()
#     simulate_sharking_thread.join()
#     # 計算最終分數
#     final_score = sim.strategy_score()
#     print(f"最終分數: {final_score}")
#
#
# if __name__ == "__main__":
#     main_thread()

