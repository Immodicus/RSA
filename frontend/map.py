from flask import Flask, render_template, request, jsonify
from flask_wtf import FlaskForm
from wtforms import SubmitField
import json, requests


# Store the drone data in a global dictionary
drone_data_live = {}


app = Flask(__name__)
app.config['SECRET_KEY'] = 'pk.eyJ1IjoieGNsb3VkIiwiYSI6ImNsZ3huZnFiZzAxOHQzcGp1YzExMHM2ZmUifQ.BMxHQXbo6vPPptp_cSAIbw'


class DroneForm(FlaskForm):
    start = SubmitField('Start',render_kw={"class": "btn btn-primary btn-md me-4 m-4 px-4"})
    stop = SubmitField('Stop ',render_kw={"class": "btn btn-danger btn-md me-4 m-4 px-4"})


@app.route('/', methods=['GET', 'POST'])
def home():
    form = DroneForm()
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