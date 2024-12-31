import threading
import time
from envs.control_world import ControlWorld



def main_thread():
    sim = ControlWorld(world_angle=3, strategy_parameters = (0.2, 0.5))
    sim.setup_environment()

    # 創建副線程來運行模擬
    simulation_thread = threading.Thread(target=sim.run_simulation)
    place_box_thread = threading.Thread(target=sim.place_box)

    # simulate_sharking_thread = threading.Thread(target=sim.simulate_sharking)
    # simulate_sharking_thread.start()
    # simulate_sharking_thread.join()

    simulation_thread.start()
    place_box_thread.start()
    place_box_thread.join()

    simulate_sharking_thread = threading.Thread(target=sim.simulate_sharking)
    simulate_sharking_thread.start()
    simulate_sharking_thread.join()
    time.sleep(30)
    # 計算最終分數
    final_score = sim.strategy_score()
    print(f"最終分數: {final_score}")

if __name__ == "__main__":
    main_thread()