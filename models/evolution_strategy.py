import numpy as np


class EvolutionStrategy:
    def __init__(self, param_bounds, population_size=50, sigma=0.1, learning_rate=0.1):
        """
        param_bounds: 每個參數的範圍 [(lower1, upper1), (lower2, upper2)]
        population_size: 種群大小
        sigma: 噪聲幅度
        learning_rate: 學習率
        """
        self.param_bounds = np.array(param_bounds)  # 每個參數的上下限
        self.param_dim = len(param_bounds)  # 參數的數量
        self.population_size = population_size  # 每代的候選解數量
        self.sigma = sigma  # 噪聲幅度
        self.learning_rate = learning_rate  # 學習率

        # 初始化參數，在範圍內隨機選擇
        self.parameters = np.array([np.random.uniform(low, high) for low, high in self.param_bounds])

    def _discretize(self):

    def get_population(self):
        """基於當前參數生成種群"""
        population = []
        for _ in range(self.population_size):
            # 為每個參數加上噪聲並裁剪到範圍內
            individual = self.parameters + self.sigma * np.random.randn(self.param_dim)
            individual = np.clip(individual, self.param_bounds[:, 0], self.param_bounds[:, 1])
            population.append(individual)
        return population

    def update_parameters(self, population, rewards):
        """根據主程序傳入的獎勵更新當前參數"""
        rewards = np.array(rewards)
        rewards_normalized = (rewards - np.mean(rewards)) / (np.std(rewards) + 1e-8)

        # 計算梯度
        gradients = np.mean([rewards_normalized[i] * (population[i] - self.parameters) / self.sigma
                             for i in range(self.population_size)], axis=0)

        # 更新參數
        self.parameters += self.learning_rate * gradients

        # 確保更新後的參數在範圍內
        self.parameters = np.clip(self.parameters, self.param_bounds[:, 0], self.param_bounds[:, 1])

    def get_parameters(self):
        """獲取當前參數"""
        return self.parameters