import socket
import _thread
import sys
import json
import requests

from flask import Flask, render_template, request
from flask_wtf import FlaskForm
from wtforms import SubmitField


drone_data: dict = {}

drone_data_live = {} # Store the drone data in a global dictionary

def client_echo_thread(client_socket: socket.socket, addr):

    while True:
        message = client_socket.recv(1024)
        if not message: break
        
        message_text = message.decode("utf-8")
        message_json = json.loads(message_text)
        drone_data[message_json['drone_id']] = message_json

        # Extract the relevant information from the simulation data
        drone_id = str(message_json['drone_id'])
        latitude = str(message_json['latitude'])
        longitude = str(message_json['longitude'])
        altitude = str(message_json['altitude'])
        heading = str(message_json['heading'])
        horizontal_velocity = str(message_json['horizontal_velocity'])
        #print(drone_data.items())
        #print(f"Received {message_text} from {addr}")
        #t = json.dumps(list(drone_data.values()), indent=4)
        #print(f'drone_data: {t}')
        #print("Drone ID: " + drone_id)
        #print("Drone latitude: " + latitude)
        #print("Drone longitude: " + longitude)
        #print("Drone Heading: " + heading)
        #print("Drone Velocity: " + horizontal_velocity)
        #print("Drone Probable collision: " + str(message_json['probable_collision_points']['0']['latitude']) + ", " + str(message_json['probable_collision_points']['0']['longitude']))

        # Create a dictionary with the drone data
        drone_data_line = {
            'drone_id': drone_id,
            'latitude': latitude,
            'longitude': longitude,
            'altitude': altitude,
            'heading': heading,
            'horizontal_velocity': horizontal_velocity
        }

        # Send a POST request to the Flask server with the drone data
        requests.post('http://localhost:8000/', json=drone_data_live)

    print(f"Closing connection to {addr}")
    client_socket.close()

  
def get_drone_data():
   pass



server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
server_socket.bind(('0.0.0.0', int(sys.argv[1])))
server_socket.listen()
print(f"Starting TCP server at port {int(sys.argv[1])}")




while True:
    client_socket, addr = server_socket.accept()
    print(f"Accepted new client at: {addr}")
    _thread.start_new_thread(client_echo_thread, (client_socket, addr))

server_socket.close()