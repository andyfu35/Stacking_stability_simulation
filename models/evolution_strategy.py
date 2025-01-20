import numpy as np


class EvolutionStrategy:
    def __init__(self, param_bounds, step_sizes, sigmas=None, population_size=50, sigma=0.1, learning_rate=0.1, init_param=None):
        """
        param_bounds: 每個參數的範圍 [(lower1, upper1), (lower2, upper2)]
        step_sizes: 每個參數的步長 [step_size1, step_size2]
        population_size: 種群大小
        sigma: 噪聲幅度
        learning_rate: 學習率
        init_param: 用於初始化的參數 (可選)
        """
        self.param_bounds = np.array(param_bounds)  # 每個參數的上下限
        self.step_sizes = np.array(step_sizes)  # 每個參數的步長
        self.param_dim = len(param_bounds)  # 參數的數量
        self.population_size = population_size  # 每代的候選解數量
        self.sigma = sigma  # 噪聲幅度
        self.learning_rate = learning_rate  # 學習率
        self.sigmas = np.array(sigmas) if sigmas is not None else np.array([0.1] * self.param_dim)
        # 如果提供了 init_param，使用該初始參數；否則隨機初始化
        if init_param is not None:
            self.parameters = np.clip(np.array(init_param), self.param_bounds[:, 0], self.param_bounds[:, 1])
        else:
            self.parameters = np.array([np.random.uniform(low, high) for low, high in self.param_bounds])

    @staticmethod
    def _discretize(values, step_sizes):
        """根據指定的步長對每個參數進行離散化"""
        values = np.around(values, decimals=5)
        return np.round(values / step_sizes) * step_sizes

    def get_population(self):
        """基於當前參數生成種群"""
        population = []
        for _ in range(self.population_size):
            # 生成噪聲
            noise = self.sigmas * np.random.randn(self.param_dim)
            # 加上噪聲並裁剪到範圍內
            individual = self.parameters + noise
            individual = np.clip(individual, self.param_bounds[:, 0], self.param_bounds[:, 1])
            # 根據各自的步長離散化
            individual = self._discretize(individual, self.step_sizes)
            population.append(individual)
        return population

    def update_parameters(self, population, rewards):
        """根據主程序傳入的獎勵更新當前參數"""
        rewards = np.array(rewards)
        rewards_normalized = (rewards - np.mean(rewards)) / (np.std(rewards) + 1e-8)

        # 計算梯度

        gradients = np.mean([rewards_normalized[i] * (population[i] - self.parameters) / self.sigma
                             for i in range(self.population_size)], axis=0)
        print(gradients)

        # 更新參數
        self.parameters += self.learning_rate * gradients
        self.parameters = np.clip(self.parameters, self.param_bounds[:, 0], self.param_bounds[:, 1])

    def get_parameters(self):
        """獲取當前參數"""
        return self.parameters