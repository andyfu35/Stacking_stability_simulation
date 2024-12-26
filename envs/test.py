import pybullet as p
import pybullet_data

# 初始化 PyBullet
p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -9.8)

# 创建中空集装箱
def create_hollow_container(container_size, wall_thickness):
    l, w, h = container_size

    # 集装箱的左侧对齐到 x=0，因此整体需要向右偏移 l / 2
    x_offset = l / 2

    # 底部
    p.createMultiBody(
        baseMass=0,
        baseCollisionShapeIndex=p.createCollisionShape(
            p.GEOM_BOX,
            halfExtents=[l / 2, w / 2, wall_thickness / 2]
        ),
        baseVisualShapeIndex=p.createVisualShape(
            p.GEOM_BOX,
            halfExtents=[l / 2, w / 2, wall_thickness / 2],
            rgbaColor=[0, 0, 1, 0.5]
        ),
        basePosition=[x_offset, 0, -h / 2 + wall_thickness / 2]
    )

    # 顶部
    p.createMultiBody(
        baseMass=0,
        baseCollisionShapeIndex=p.createCollisionShape(
            p.GEOM_BOX,
            halfExtents=[l / 2, w / 2, wall_thickness / 2]
        ),
        baseVisualShapeIndex=p.createVisualShape(
            p.GEOM_BOX,
            halfExtents=[l / 2, w / 2, wall_thickness / 2],
            rgbaColor=[0, 0, 1, 0.5]
        ),
        basePosition=[x_offset, 0, h / 2 - wall_thickness / 2]
    )

    # 左侧（对齐到 x=0）
    p.createMultiBody(
        baseMass=0,
        baseCollisionShapeIndex=p.createCollisionShape(
            p.GEOM_BOX,
            halfExtents=[wall_thickness / 2, w / 2, h / 2]
        ),
        baseVisualShapeIndex=p.createVisualShape(
            p.GEOM_BOX,
            halfExtents=[wall_thickness / 2, w / 2, h / 2],
            rgbaColor=[0, 0, 1, 0.5]
        ),
        basePosition=[wall_thickness / 2, 0, 0]
    )

    # 背侧
    p.createMultiBody(
        baseMass=0,
        baseCollisionShapeIndex=p.createCollisionShape(
            p.GEOM_BOX,
            halfExtents=[l / 2, wall_thickness / 2, h / 2]
        ),
        baseVisualShapeIndex=p.createVisualShape(
            p.GEOM_BOX,
            halfExtents=[l / 2, wall_thickness / 2, h / 2],
            rgbaColor=[0, 0, 1, 0.5]
        ),
        basePosition=[x_offset, -w / 2 + wall_thickness / 2, 0]
    )

    # 前方（添加面板）
    p.createMultiBody(
        baseMass=0,
        baseCollisionShapeIndex=p.createCollisionShape(
            p.GEOM_BOX,
            halfExtents=[l / 2, wall_thickness / 2, h / 2]
        ),
        baseVisualShapeIndex=p.createVisualShape(
            p.GEOM_BOX,
            halfExtents=[l / 2, wall_thickness / 2, h / 2],
            rgbaColor=[0, 0, 1, 0.5]
        ),
        basePosition=[x_offset, w / 2 - wall_thickness / 2, 0]
    )


# 创建一个 6x2.35x2.36 的中空集装箱，壁厚为 0.1
create_hollow_container(container_size=(6.0, 2.35, 2.36), wall_thickness=0.1)

# 开始模拟
while True:
    p.stepSimulation()