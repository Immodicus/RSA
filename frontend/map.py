from flask import Flask, render_template, request, jsonify
from flask_wtf import FlaskForm
from wtforms import SubmitField
import json, requests
import subprocess, os
from signal import SIGKILL


# Store the drone data in a global dictionary
drone_data_live = {}


app = Flask(__name__)
app.config['SECRET_KEY'] = 'pk.eyJ1IjoieGNsb3VkIiwiYSI6ImNsZ3huZnFiZzAxOHQzcGp1YzExMHM2ZmUifQ.BMxHQXbo6vPPptp_cSAIbw'

simulation_process: subprocess.Popen = None  # Variable to store the simulation process

class DroneForm(FlaskForm):
    start = SubmitField('Start',render_kw={"class": "btn btn-primary btn-md me-4 m-4 px-4", "id": "startButton"})
    stop = SubmitField('Stop ',render_kw={"class": "btn btn-danger btn-md me-4 m-4 px-4"})


@app.route('/', methods=['GET', 'POST'])
def home():
    form = DroneForm()

    if request.method == 'POST':
        global simulation_process
        global drone_data_live
        
        if 'start' in request.form:
            print('start')
            if simulation_process == None:
                simulation_process = subprocess.Popen(["python3", "main.py", "-simulation_file", "simulation.json"], cwd='../', preexec_fn=os.setsid) 

        elif 'stop' in request.form:
            print('stop')
            if simulation_process != None:
                os.killpg(os.getpgid(simulation_process.pid), SIGKILL)
                simulation_process = None
                drone_data_live = {}

    return render_template('home.html', form=form)


@app.route('/drone-data', methods=['GET', 'POST'])
def drone_data_func():
    global drone_data_live
    
    if request.method == 'POST':
        drone_data_live = request.get_json()
        
        return 'Ok', 200
    
    return drone_data_live, 200


if __name__ == '__main__':
    app.run(host='localhost', port=8000, debug=True)