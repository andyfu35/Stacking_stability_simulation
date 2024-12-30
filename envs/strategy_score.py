import pybullet as p
from math import degrees

# 最終打分函數
def calculate_final_score():
    """
    計算最終堆疊的穩定性分數。
    - 如果有箱子倒塌，總分為 -5。
    - 如果有箱子掉落，總分為 -5。
    - 如果沒有倒塌或掉落，根據箱子的平均傾斜度計算分數。
      每 1 度扣 1 分，滿分 10 分。

    返回總分和詳情。
    """
    num_bodies = p.getNumBodies()
    total_score = 10  # 初始總分為 10 分
    box_scores = []
    total_roll = 0
    total_pitch = 0
    valid_boxes = 0

    for i in range(num_bodies):
        body_id = p.getBodyUniqueId(i)

        # 判斷是否為箱子（忽略質量為 0 的靜態物體，如集裝箱）
        mass, _, _ = p.getDynamicsInfo(body_id, -1)[:3]
        if mass <= 0:
            continue

        # 獲取位置、方向
        position, orientation = p.getBasePositionAndOrientation(body_id)

        # 計算傾斜角度（Roll 和 Pitch）
        roll, pitch, _ = map(degrees, p.getEulerFromQuaternion(orientation))

        # 判斷是否倒塌或掉落
        is_fallen = abs(roll) > 45 or abs(pitch) > 45 or position[2] < 0.1
        if is_fallen:
            return -5, []  # 倒塌或掉落直接返回 -5 分

        # 累計有效箱子的傾斜角度
        total_roll += abs(roll)
        total_pitch += abs(pitch)
        valid_boxes += 1

        # 保存箱子的狀態
        box_scores.append({
            "id": body_id,
            "position": position,
            "roll": roll,
            "pitch": pitch,
            "is_fallen": is_fallen
        })

    # 如果沒有有效的箱子，分數為 0
    if valid_boxes == 0:
        return 0, box_scores

    # 計算平均傾斜度
    avg_tilt = (total_roll + total_pitch) / (2 * valid_boxes)

    # 根據平均傾斜度扣分
    tilt_penalty = avg_tilt // 1  # 每 2 度扣 1 分
    final_score = max(10 - tilt_penalty, 0)  # 總分不低於 0

    return final_score, box_scores

# 使用範例：計算最終分數
final_score, box_scores = calculate_final_score()

# 打印結果
print(f"最終總分: {final_score}")
# print("各箱子分數:")
# for box in box_scores:
#     print(f"箱子 {box['id']} - 位置: {box['position']}")
#     print(f"  傾斜角度: Roll={box['roll']}, Pitch={box['pitch']}")
