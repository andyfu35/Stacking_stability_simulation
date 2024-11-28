import pybullet as p
import pybullet_data
import time

# 初始化 PyBullet
p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -9.8)

# 加载地板
p.loadURDF("plane.urdf")

# 加载柔体
try:
    soft_body_id = p.loadSoftBody(
        fileName="sealed_soft_cube.obj",  # 柔体模型
        basePosition=[0, 0, 100],        # 初始高度增加，确保柔体在地板上方
        scale=1.0,
        mass=1.0,
        useNeoHookean=1,
        useBendingSprings=1,
        springElasticStiffness=5,       # 降低弹性参数
        springDampingStiffness=0.3,
        springBendingStiffness=0.1,     # 降低弯曲参数
        collisionMargin=0.1,            # 增大碰撞边界
        useSelfCollision=1,
        frictionCoeff=0.5
    )
    print("柔体加载成功")
except Exception as e:
    print(f"柔体加载失败: {e}")
    p.disconnect()
    exit()

# 预热模拟，让柔体与地板分离
for _ in range(100):  # 模拟 100 步
    p.stepSimulation()

# 模拟正式运行
try:
    for step in range(240):  # 运行 240 步
        p.stepSimulation()
        time.sleep(1 / 240)  # 默认模拟频率
except Exception as e:
    print(f"模拟中出现错误：{e}")
finally:
    p.disconnect()
    print("模拟完成")
