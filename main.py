import argparse
import json
from drone import Drone
from multiprocessing import Process

def main():
    argParser = argparse.ArgumentParser()
    argParser.add_argument('-simulation_file', '--simulation_file', type=str)
    
    args = argParser.parse_args()
    print(f'Simulation file is: {args.simulation_file}')

    json_str = ''
    try:
        f = open(args.simulation_file, 'r')
        json_str = f.read()
    except:
        print(f'Failed to open file {args.simulation_file}')

    sim_json = json.loads(json_str)
    print(json_str)
    
    drone_processes: list[Process] = []
    for drone in sim_json['drones']:
        drone_processes.append(Process(target=run_drone, args=(sim_json['settings'], drone,)))

    for drone_process in drone_processes:
        drone_process.start()

    for drone_process in drone_processes:
        drone_process.join()

    print(f'All drones done!')

def run_drone(settings, drone_data):
    drone_obj = Drone(settings, drone_data)
    drone_obj.start()
    drone_obj.join()


if __name__ == '__main__':
    main()