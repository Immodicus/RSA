from flask import Flask, render_template, request, jsonify
from flask_wtf import FlaskForm
from wtforms import SubmitField
import json, requests


# Store the drone data in a global dictionary
#drone_data_live = {}


app = Flask(__name__)
app.config['SECRET_KEY'] = 'pk.eyJ1IjoieGNsb3VkIiwiYSI6ImNsZ3huZnFiZzAxOHQzcGp1YzExMHM2ZmUifQ.BMxHQXbo6vPPptp_cSAIbw'


class DroneForm(FlaskForm):
    start = SubmitField('Start',render_kw={"class": "btn btn-primary btn-md me-4 m-4 px-4"})
    stop = SubmitField('Stop ',render_kw={"class": "btn btn-danger btn-md me-4 m-4 px-4"})


@app.route('/', methods=['GET', 'POST'])
def home():
    form = DroneForm()
    drone_data_live = {}
    #latitude = 0
    #longitude = 0
    if request.method == 'POST' and request.content_type == 'application/json':
        drone_data_live = request.get_json()
        #drone_id = drone_data_live['drone_id']
        #drone_data_live[drone_id] = drone_data_live
        #latitude = drone_data_live[drone_id]['latitude']
        #longitude = drone_data_live[drone_id]['longitude']
        #print(latitude, longitude)
    #return "<h1>Latitude: " + str(latitude) + " Longitude: " + str(longitude) + "</h1>"
    return render_template('home.html', form=form, drone_data_live=drone_data_live)


'''
@app.route('/drone', methods=['POST'])
def drone():
    if request.form['start']:
        # Start the drone
        pass
        
    elif request.form['stop']:
        # stop the drone
        pass
    return 'OK'

    # Get the drone data from the POST request
    drone_data_live = request.get_json()

    # Store the drone data in the global dictionary
    drone_id = drone_data_live['drone_id']
    drone_data_live[drone_id] = drone_data_live

    # Send a JSON response with the success message
    response = {'success': True}
    return jsonify(response)
'''
if __name__ == '__main__':
    app.run(host='localhost', port=8000, debug=True)