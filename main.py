import threading
from envs.control_world import ControlWorld
from models.evolution_strategy import EvolutionStrategy
from utils.visualization import ParameterPlotter


param_bounds = [(0.0, 0.2), (0.0, 1.0)]
step_sizes = [0.01, 0.05]


sigmas = [0.02, 0.1]
learning_rate = 0.1
population_size = 10
generations = 100
init_param = [0.1, 0.5]

def main_thread():
    sim = ControlWorld(world_angle=2)
    plotter = ParameterPlotter(output_file="utils/parameters_vs_iterations.png")
    sim.setup_environment()
    es = EvolutionStrategy(param_bounds, step_sizes, sigmas, population_size, learning_rate, init_param)
    simulation_thread = threading.Thread(target=sim.run_simulation)
    simulation_thread.start()
    current_time = 1
    param_data = [[], [], [], []]
    for generation in range(generations):
        population = es.get_population()
        rewards = []
        for individual in population:
            sim.update_strategy_parameters(individual)
            print("------------------------------------------")
            print("Distance : " + str(individual[0]))
            print("Power : " + str(individual[1]))
            sim.reset_world()
            sim.place_box()
            # sim.simulate_sharking()
            reward = sim.strategy_score()
            print(f"最終分數: {reward}")
            rewards.append(reward)
            param_data[0].append(current_time)
            param_data[1].append(individual[0]*20)
            param_data[2].append(individual[1]*10)
            param_data[3].append(reward)
            current_time = current_time + 1

        plotter.update_plot(param_data)
        plotter.save_plot()
        es.update_parameters(population, rewards)


    final_params = es.get_parameters()
    print("Best params:", final_params)

if __name__ == "__main__":
    main_thread()