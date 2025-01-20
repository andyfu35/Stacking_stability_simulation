import matplotlib.pyplot as plt


class ParameterPlotter:
    def __init__(self, output_file="plot.png"):
        """初始化绘图对象"""
        self.output_file = output_file
        self.fig, self.ax = plt.subplots(figsize=(10, 6))
        self.lines = {
            "param1": self.ax.plot([], [], label="Distance", marker='o')[0],
            "param2": self.ax.plot([], [], label="Power", marker='s')[0],
            "param3": self.ax.plot([], [], label="Reward", marker='^')[0],
        }
        self.ax.set_title("Parameters vs Iterations", fontsize=16)
        self.ax.set_xlabel("Iterations", fontsize=14)
        self.ax.set_ylabel("Parameter Value", fontsize=14)
        self.ax.legend()
        self.ax.grid(True)
        plt.tight_layout()
        plt.ion()  # 启用交互模式

    def update_plot(self, param):
        """更新图表数据"""
        print(param)
        self.lines["param1"].set_data(param[0], param[1])
        self.lines["param2"].set_data(param[0], param[2])
        self.lines["param3"].set_data(param[0], param[3])

        # 更新轴范围
        self.ax.set_ylim(
            min(min(param[1]), min(param[2]), min(param[3])) - 0.1,
            max(max(param[1]), max(param[2]), max(param[3])) + 0.1
        )
        self.ax.set_xlim(0, max(param[0]) + 1)

        # 刷新图表
        self.fig.canvas.draw()
        self.fig.canvas.flush_events()

    def save_plot(self):
        """保存图表到文件"""
        plt.ioff()  # 关闭交互模式
        self.fig.savefig(self.output_file)
        print(f"图表已保存到 {self.output_file}")


