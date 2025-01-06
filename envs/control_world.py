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
        self.sharking_degree = 30
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
        for x_ in range(3):
            for col in range(self.max_col):
                current_depth = self.place_strategy(col)
                box_x = self.box_info[0] / 2 + self.strategy_parameters[0] + (self.box_info[0] + 0.001) * x_ - current_depth
                box_z = self.box_info[2] / 2 + self.box_info[2] * col + self.drop_height
                for row in range(self.max_row):
                    box_y = self.container_size[1] / 2 - 0.001 - self.box_info[1] / 2 - (self.box_info[1] + 0.001) * row
                    self.boxes_position.append((box_x, box_y, box_z))
        return self.boxes_position

    def place_box(self):
        box_pos_list = self.generate_box_positions()
        # n = 0
        # n_ = 0
        for box in box_pos_list:
            # if n % 90 == 0:
            #     self.simulate_sharking()
            #     n_ += 1
            # n += 1
            create_box(self.box_info, box)
            time.sleep(0.1)

    @staticmethod
    def wait_until_stable(threshold=0.01, check_interval=0.5, max_wait=100):
        start_time = time.time()
        while time.time() - start_time < max_wait:
            all_stable = True
            for body_id in range(p.getNumBodies()):
                linear_velocity, angular_velocity = p.getBaseVelocity(body_id)
                # print(linear_velocity, "    ", angular_velocity)
                if any(abs(v) > threshold for v in linear_velocity) or any(
                        abs(w) > threshold for w in angular_velocity):
                    all_stable = False
                    break
            # print("Still moving")
            if all_stable:
                return True
            time.sleep(check_interval)
        return False

    def simulate_sharking(self):
        shaking_amplitude = np.radians(self.sharking_degree)
        sharking_amplitude_ = self.gravity_magnitude * np.sin(shaking_amplitude)
        sharking = [(sharking_amplitude_, 0, -self.gravity_z),
                    (-sharking_amplitude_, 0, -self.gravity_z)]
        for times in range(10):
            for shark in sharking:
                p.setGravity(shark[0], shark[1], shark[2])
                time.sleep(0.5)
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
        if not self.wait_until_stable():
            print("NOOOOOOOOOO")

        num_bodies = p.getNumBodies()
        total_roll = 0
        valid_boxes = 0
        penalty = 0

        for body in range(num_bodies):
            body_id = p.getBodyUniqueId(body)
            mass, _, _ = p.getDynamicsInfo(body_id, -1)[:3]
            if mass <= 0:
                continue

            position, orientation = p.getBasePositionAndOrientation(body_id)
            roll, _, _ = map(degrees, p.getEulerFromQuaternion(orientation))
            is_fallen = abs(roll) > 45 or position[2] < 0.1
            if is_fallen:
                return -5

            total_roll += roll
            valid_boxes += 1

        if valid_boxes == 0:
            return 0

        avg_roll = total_roll / valid_boxes

        if avg_roll > 0:  # 向 +x 傾斜
            penalty = (avg_roll / 0.5) * 4
        elif avg_roll < 0:
            penalty = (abs(avg_roll) / 0.5) * 2
        final_score = 10 - (self.strategy_parameters() * 10)
        final_score = max(final_score - penalty, 0)
        return final_score