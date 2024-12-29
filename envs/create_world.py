import pybullet as p
import pybullet_data
import trimesh
import numpy as np
import time
from object_create import create_box, create_container



angle = 0
gravity_magnitude = 9.8
angle = np.radians(angle)
gravity_x = gravity_magnitude * np.sin(angle)
gravity_z = gravity_magnitude * np.cos(angle)




p.connect(p.GUI)  # 使用 GUI 模式
p.setAdditionalSearchPath(pybullet_data.getDataPath())  # 加载资源路径
p.configureDebugVisualizer(p.COV_ENABLE_SHADOWS, 0)
p.setGravity(gravity_x, 0, -gravity_z)

p.setPhysicsEngineParameter(
   fixedTimeStep=1 / 100,  # 高精度时间步长
   numSolverIterations=150,  # 增加求解器迭代次数
   contactSlop=0.0001,  # 减小接触容差
   useSplitImpulse=True,
   splitImpulsePenetrationThreshold=-0.02
)

box_size = (0.213, 0.256, 0.113)

box_pos_list = []
create_container((2.0, 2.35, 2.36))
drop_height = 0.01

y_list = []
for y_value in range(5):
   if y_value == 0:
      y_list.append(y_value)
   else:
      y_list.append(y_value * box_size[1]+0.001)
      y_list.append(-y_value * box_size[1]+0.001)


for x in range(1):
   for z in range(3):
      for y in sorted(y_list, reverse=True):
         box_pos_list.append(
            (
               # box_size[0] / 2 + 0.04 + box_size[0] * x + 0.2,
             box_size[0] / 2 + 0.04 + box_size[0] * x + 0.4 - z * 0.02,
             y,
             box_size[2]/2 + (box_size[2]+0.01) * z + drop_height)
         )

# for box_pos in box_pos_list:
#    create_box(box_size, box_pos, 5)

step_time = 50
sim_time = int(len(box_pos_list) / 3 * step_time + 100)
print(sim_time)
for n in range(sim_time):
   p.stepSimulation()
   time.sleep(1 / 1000)
   if n%50 == 0:
      try:
         index = int(round(n/50))
         create_box(box_size, box_pos_list[index * 3], 5)
         create_box(box_size, box_pos_list[index * 3 + 1], 5)
         create_box(box_size, box_pos_list[index * 3 + 2], 5)

      except:
         pass

from strategy_score import calculate_final_score
calculate_final_score()




p.disconnect()