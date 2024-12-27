import pybullet as p
import pybullet_data
import time

# 初始化仿真
p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -9.8)
p.setTimeStep(0.01)
p.setPhysicsEngineParameter(numSolverIterations=100)

# 创建地面
plane_id = p.loadURDF("plane.urdf")
p.changeDynamics(plane_id, -1, lateralFriction=1.0)

# 创建箱子
box_half_extents = [0.5, 0.5, 0.5]
box_collision_shape = p.createCollisionShape(p.GEOM_BOX, halfExtents=box_half_extents)
box_visual_shape = p.createVisualShape(p.GEOM_BOX, halfExtents=box_half_extents, rgbaColor=[1, 0, 0, 1])

for i in range(15):
    z_position = 0.5 + i * 1.01
    box_id = p.createMultiBody(
        baseMass=1,
        baseCollisionShapeIndex=box_collision_shape,
        baseVisualShapeIndex=box_visual_shape,
        basePosition=[0, 0, z_position]
    )
    p.changeDynamics(box_id, -1, lateralFriction=1.0, restitution=0.0)

# 运行仿真
for _ in range(10000):
    p.stepSimulation()
    time.sleep(0.01)

p.disconnect()
