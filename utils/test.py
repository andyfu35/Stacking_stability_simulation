import matplotlib

matplotlib.use('Agg')  # 使用非交互式后端
import matplotlib.pyplot as plt

# 示例数据
x = [1, 2, 3, 4, 5]
y = [2, 3, 5, 7, 11]

# 绘图
plt.plot(x, y, label="Sample Data")
plt.xlabel("X-axis")
plt.ylabel("Y-axis")
plt.title("Static Plot Example")
plt.legend()

# 保存为文件
plt.savefig("plot.png")