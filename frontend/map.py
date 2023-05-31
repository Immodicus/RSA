from flask import Flask, render_template, request
import subprocess, os
import glob
from signal import SIGKILL


# Store the drone data in a global dictionary
drone_data_live = {}


app = Flask(__name__)
app.config['SECRET_KEY'] = 'pk.eyJ1IjoieGNsb3VkIiwiYSI6ImNsZ3huZnFiZzAxOHQzcGp1YzExMHM2ZmUifQ.BMxHQXbo6vPPptp_cSAIbw'

simulation_process: subprocess.Popen = None  # Variable to store the simulation process


@app.route('/', methods=['GET', 'POST'])
def home():
    return render_template('home.html')

@app.route('/start', methods=['POST'])
def start():
    global simulation_process
    
    json = request.get_json()
    if "sim_file" not in json:
        return '???', 400
    
    if simulation_process == None:
        simulation_process = subprocess.Popen(["python3", "main.py", "-simulation_file", json["sim_file"]], cwd='../simulation/', preexec_fn=os.setsid)

    return 'Ok', 200

@app.route('/stop', methods=['POST'])
def stop():
    global simulation_process
    global drone_data_live
    
    if simulation_process != None:
        os.killpg(os.getpgid(simulation_process.pid), SIGKILL)
        simulation_process = None
        drone_data_live = {}

    return 'Ok', 200

@app.route('/list', methods=['GET'])
def list():
    simulation_file_list = glob.glob('../simulation/simulation*.json')
    simulation_file_list = [sim_file.removeprefix('../simulation/') for sim_file in simulation_file_list]

    return {'sim_files': simulation_file_list}, 200

@app.route('/drone-data', methods=['GET', 'POST'])
def drone_data_func():
    global drone_data_live
    
    if request.method == 'POST':
        drone_data_live = drone_data_live | request.get_json()
        
        return 'Ok', 200
    
    return drone_data_live, 200


if __name__ == '__main__':
    app.run(host='localhost', port=8000, debug=True)