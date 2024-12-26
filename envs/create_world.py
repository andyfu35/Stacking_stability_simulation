import pybullet as p
import pybullet_data
import trimesh
import numpy as np
import time

from object_create import create_box, create_container
options = "--useGpuPipeline"

params = p.getPhysicsEngineParameters()
print("当前物理引擎参数:", params)

# 检查是否启用了 GPU 管道
if params.get("useGpuPipeline", False):
   print("GPU 管道已启用")
else:
   print("GPU 管道未启用")

p.connect(p.GUI, options=options)  # 使用 GUI 模式
p.setAdditionalSearchPath(pybullet_data.getDataPath())  # 加载资源路径
p.setGravity(0, 0, -9.8)  # 设置重力
p.setPhysicsEngineParameter(
   fixedTimeStep=1 / 500,  # 高精度时间步长
   numSolverIterations=100,  # 增加求解器迭代次数
   contactSlop=0.01  # 减小接触容差
)

box_size = (0.213, 0.256, 0.113)

box_pos_list = []
create_container((6.0, 2.35, 2.36))
drop_height = 0.01

y_list = []
for y_value in range(5):
   if y_value == 0:
      y_list.append(y_value)
   else:
      y_list.append(y_value * box_size[1]+0.001)
      y_list.append(-y_value * box_size[1]+0.001)


for x in range(1):
   for z in range(20):
      for y in sorted(y_list, reverse=True):
         box_pos_list.append(
            (box_size[0]/2 + 0.04 + box_size[0] * x + 0.005,
             y,
             box_size[2]/2 + box_size[2] * z + drop_height)
         )

for box_pos in box_pos_list:
   create_box(box_size, box_pos, 5)


for n in range(20000):
   p.stepSimulation()
   time.sleep(1 / 480)
   # if n%100 == 0:
   #    index = int(round(n/100))
   #    create_box(box_size, box_pos_list[index*3], 0)
   #    create_box(box_size, box_pos_list[index*3+1], 0)
   #    create_box(box_size, box_pos_list[index*3+2], 0)



p.disconnect()