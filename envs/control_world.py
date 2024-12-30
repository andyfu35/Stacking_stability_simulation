import pybullet as p
import pybullet_data
import trimesh
import numpy as np
import time

from numpy.lib.function_base import place
from object_create import create_box, create_container


class ControlWorld:
    def __init__(self, world_angle=0, box_info=(0.213, 0.255, 0.113, 5), strategy_parameters=(0.01, 0.5)):
        self.world_angle = np.radians(world_angle)
        self.box_info = box_info
        self.drop_height = 0.005
        self.strategy_parameters = strategy_parameters
        self.gravity_magnitude = 9.81
        self.gravity_x = self.gravity_magnitude * np.sin(self.world_angle)
        self.gravity_z = self.gravity_magnitude * np.cos(self.world_angle)
        self.container_size = (3.0, 2.35, 2.36)
        self.physics_client = p.connect(p.GUI)
        self.max_col = int((self.container_size[2] - self.drop_height) / self.box_info[2])
        self.max_row = int((self.container_size[1] - 0.005) / self.box_info[1])
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

    def place_strategy(self, current_col = 0):
        return self.strategy_parameters[0] * (current_col / self.max_col) ** self.strategy_parameters[1]

    def generate_box_positions(self):
        for col in range(self.max_col):
            current_depth = self.place_strategy(col)
            box_x = self.box_info[0] / 2 + self.strategy_parameters[0] - current_depth
            box_z = self.box_info[2] / 2 + self.box_info[2] * col + self.drop_height
            for row in range(self.max_row):
                # if col % 2 == 0:
                #     box_y = self.container_size[1] / 2 - 0.001 - self.box_info[1] / 2 - self.box_info[1] * row
                # else:
                #     box_y = -self.container_size[1] / 2 + 0.001 + self.box_info[1] / 2 + self.box_info[1] * (self.max_row - row - 1)
                box_y = self.container_size[1] / 2 - 0.001 - self.box_info[1] / 2 - self.box_info[1] * row
                self.boxes_position.append((box_x, box_y, box_z))
        print(len(self.boxes_position))
        return self.boxes_position

    def run_simulation(self, step_time = 50):
        box_pos_list = self.generate_box_positions()
        sim_time = int(len(box_pos_list) * step_time / 3 + 100)

        for n in range(sim_time):
            p.stepSimulation()
            time.sleep(1/1000)

            if n % 50 == 0:
                index = int(round(n / 50))
                try:
                    create_box(self.box_info, box_pos_list[index * 3])
                    create_box(self.box_info, box_pos_list[index * 3 + 1])
                    create_box(self.box_info, box_pos_list[index * 3 + 2])

                except IndexError:
                    pass

    # def

    @staticmethod
    def close_windows():
        p.disconnect()

if __name__ == "__main__":
    sim = ControlWorld(world_angle=3)
    sim.setup_environment()
    sim.run_simulation()
    # sim.close_windows()