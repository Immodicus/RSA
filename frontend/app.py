from flask import Flask, render_template, request
from flask_wtf import FlaskForm
from wtforms import SubmitField
import requests


app = Flask(__name__)
app.config['SECRET_KEY'] = 'pk.eyJ1IjoieGNsb3VkIiwiYSI6ImNsZ3huZnFiZzAxOHQzcGp1YzExMHM2ZmUifQ.BMxHQXbo6vPPptp_cSAIbw'


class DroneForm(FlaskForm):
    start = SubmitField('Start',render_kw={"class": "btn btn-primary btn-md me-4 m-4 px-4"})
    stop = SubmitField('Stop ',render_kw={"class": "btn btn-danger btn-md me-4 m-4 px-4"})


@app.route('/', methods=['GET', 'POST'])
def home():
    form = DroneForm()
    return render_template('home.html', form=form)


@app.route('/drone', methods=['POST'])
def drone():
    if request.form['start']:
        # Start the drone
        pass
        
    elif request.form['stop']:
        # stop the drone
        pass
    return 'OK'

if __name__ == '__main__':
    app.run(debug=True)