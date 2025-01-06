import threading
import yaml
from envs.control_world import ControlWorld
from models.evolution_strategy import EvolutionStrategy

# with open("configs/configuration.yaml", "r") as f:
#     config = yaml
param_bounds = [(0.0, 0.2), (0.0, 1.0)]

sigma = 0.1
learning_rate = 0.1
population_size = 10
generations = 500


def main_thread():
    sim = ControlWorld(world_angle=2)
    sim.setup_environment()
    es = EvolutionStrategy(param_bounds, population_size, sigma, learning_rate)
    simulation_thread = threading.Thread(target=sim.run_simulation)
    simulation_thread.start()

    for generation in range(generations):
        population = es.get_population()

        rewards = []
        for individual in population:
            sim.update_strategy_parameters(individual)
            print(individual)
            sim.reset_world()
            sim.place_box()
            sim.simulate_sharking()
            reward = sim.strategy_score()
            print(f"最終分數: {reward}")
            rewards.append(reward)

        es.update_parameters(population, rewards)

    final_params = es.get_parameters()
    print("Best params:", final_params)


            # reset_world = threading.Thread(target=sim.reset_world)
            # reset_world.start()
            # reset_world.join()
            #
            # place_box_thread = threading.Thread(target=sim.place_box)
            # place_box_thread.start()
            # place_box_thread.join()
            #
            # simulate_sharking_thread = threading.Thread(target=sim.simulate_sharking)
            # simulate_sharking_thread.start()
            # simulate_sharking_thread.join()

        # final_score = sim.strategy_score()
        # print(f"最終分數: {final_score}")

if __name__ == "__main__":
    main_thread()