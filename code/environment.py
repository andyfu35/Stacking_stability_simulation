import pybullet as p
import pybullet_data
import time
import math

class BoxStackingSimulator:
    def __init__(self, box_info):
        # 初始化 PyBullet 模擬環境
        self.camera_distance = 5
        self.camera_yaw = -90
        self.camera_pitch = -30
        self.tilt_angle = -3
        self.place_height = 0.1
        self.half_extents = [box_info[0] / 2,  # Length / 2
                             box_info[1] / 2,  # Width / 2
                             box_info[2] / 2]
        self.box_mass = box_info[3]

        # 連接到 PyBullet
        p.connect(p.GUI)  # 使用 GUI 視覺化
        p.setAdditionalSearchPath(pybullet_data.getDataPath())  # 加載默認資源
        p.setGravity(0, 0, -9.8)  # 設置重力

        # 設置攝像機
        self.set_camera()

    def set_camera(self):
        p.resetDebugVisualizerCamera(
            cameraDistance=self.camera_distance,    # 與場景中心的距離
            cameraYaw=self.camera_yaw,              # 水平旋轉角度 (0 表示正前方)
            cameraPitch=self.camera_pitch,          # 垂直旋轉角度 (0 表示正水平)
            cameraTargetPosition=[0, 0, 0]          # 目標點 (場景中心)
        )

    def create_floor(self):
        # 創建灰色地板
        ground_collision = p.createCollisionShape(p.GEOM_BOX, halfExtents=[5, 5, 0.1])  # 地板大小
        ground_visual = p.createVisualShape(p.GEOM_BOX, halfExtents=[5, 5, 0.1], rgbaColor=[0.5, 0.5, 0.5, 1])  # 灰色
        ground_body = p.createMultiBody(baseMass=0, baseCollisionShapeIndex=ground_collision, baseVisualShapeIndex=ground_visual)

        # 重置地板的位置和傾斜角度
        p.resetBasePositionAndOrientation(ground_body, [0, 0, 0], p.getQuaternionFromEuler([0, math.radians(self.tilt_angle), 0]))

    def create_wall(self):
        # 創建牆壁參數
        wall_thickness = 0.2  # 牆壁厚度
        wall_height = 2  # 牆壁高度
        wall_length = 3  # 牆壁長度

        # 創建牆壁碰撞形狀
        wall_collision_1 = p.createCollisionShape(p.GEOM_BOX,
                                                  halfExtents=[wall_length / 2, wall_thickness / 2, wall_height / 2])
        wall_collision_2 = p.createCollisionShape(p.GEOM_BOX, halfExtents=[3.0, wall_thickness / 2, wall_height / 2])

        # 創建牆壁可視形狀，統一設置為紅色
        wall_visual_1 = p.createVisualShape(p.GEOM_BOX,
                                            halfExtents=[wall_length / 2, wall_thickness / 2, wall_height / 2],
                                            rgbaColor=[1, 0, 0, 1])
        wall_visual_2 = p.createVisualShape(p.GEOM_BOX, halfExtents=[3.0, wall_thickness / 2, wall_height / 2],
                                            rgbaColor=[1, 0, 0, 1])

        # 創建牆壁剛體
        wall_body_1 = p.createMultiBody(baseMass=0, baseCollisionShapeIndex=wall_collision_1,
                                        baseVisualShapeIndex=wall_visual_1)
        wall_body_2 = p.createMultiBody(baseMass=0, baseCollisionShapeIndex=wall_collision_2,
                                        baseVisualShapeIndex=wall_visual_2)
        wall_body_3 = p.createMultiBody(baseMass=0, baseCollisionShapeIndex=wall_collision_2,
                                        baseVisualShapeIndex=wall_visual_2)

        # 設置旋轉角度
        rotation_quaternion_1 = p.getQuaternionFromEuler([math.radians(self.tilt_angle), 0, math.pi/2])  # 前牆旋轉
        rotation_quaternion_2 = p.getQuaternionFromEuler([0, math.radians(self.tilt_angle), 0])  # 側牆旋轉

        # --- 前牆 ---
        p.resetBasePositionAndOrientation(
            wall_body_1,
            [wall_thickness / 2, 0, wall_height / 2],  # x 軸靠近地板正前方
            rotation_quaternion_1
        )

        # --- 左側牆 ---
        p.resetBasePositionAndOrientation(
            wall_body_2,
            [-3.0 + wall_thickness / 2, (wall_length + wall_thickness / 2) / 2, wall_height / 2],  # 偏移 x 和 y
            rotation_quaternion_2
        )

        # --- 右側牆 ---
        p.resetBasePositionAndOrientation(
            wall_body_3,
            [-3.0 + wall_thickness / 2, -(wall_length + wall_thickness / 2) / 2, wall_height / 2],  # 偏移 x 和 y
            rotation_quaternion_2
        )

    def create_rigid_body(self, base_position):
        # 剛體框架
        collision = p.createCollisionShape(p.GEOM_BOX, halfExtents=[dim / 2 for dim in self.box_size])
        visual = p.createVisualShape(p.GEOM_BOX, halfExtents=[dim / 2 for dim in self.box_size],
                                     rgbaColor=[0.8, 0.3, 0.3, 1])
        rigid_body = p.createMultiBody(baseMass=1, baseCollisionShapeIndex=collision,
                                       baseVisualShapeIndex=visual, basePosition=base_position)
        return rigid_body

    def create_soft_body_shell(self, base_position):
        # 柔體外殼
        soft_body = p.loadSoftBody(
            fileName=None, basePosition=base_position, mass=0.5,
            useNeoHookean=1, useBendingSprings=1, springElasticStiffness=40,
            springDampingStiffness=0.3, springBendingStiffness=0.1,
            collisionMargin=0.02, useSelfCollision=1, frictionCoeff=0.5
        )
        return soft_body

    def create_box(self, position=5):
        box_collision = p.createCollisionShape(p.GEOM_BOX, halfExtents=self.half_extents)
        box_visual = p.createVisualShape(p.GEOM_BOX, halfExtents=self.half_extents, rgbaColor=[0, 0, 1, 1])

        # 設置箱子的旋轉角度，與地面相同
        rotation_quaternion = p.getQuaternionFromEuler([0, math.radians(self.tilt_angle), 0])

        # 創建箱子並設置位置和旋轉
        p.createMultiBody(
            baseMass=self.box_mass,
            baseCollisionShapeIndex=box_collision,
            baseVisualShapeIndex=box_visual,
            basePosition=[-self.half_extents[0], self.half_extents[1]*position*2, self.half_extents[2] + self.place_height],  # 放置在地面上
            baseOrientation=rotation_quaternion  # 設置旋轉
        )

    def run_simulation(self):
        # 運行模擬
        p.setTimeStep(1 / 60)
        p.setRealTimeSimulation(1)  # 啟用實時模擬模式

        for i in range(5):
            self.create_box(i)
            time.sleep(1)
            self.create_box(-   i)
            time.sleep(1)

        # 斷開 PyBullet 連接
        p.disconnect()

    def setup_scene(self):
        # 設置場景
        self.create_floor()
        self.create_wall()

    def start(self):
        # 設置場景並啟動模擬
        self.setup_scene()
        self.run_simulation()

# 使用類進行模擬
box_info = [0.213, 0.255, 0.113, 5]  # 長、寬、高和質量
simulator = BoxStackingSimulator(box_info)
simulator.start()
