import threading
import json
import time
import paho.mqtt.client as mqtt

from math import sqrt, atan2, pi, sin, cos, degrees, ceil
from time import sleep
from haversine import haversine, Unit, inverse_haversine, Direction
from sympy.geometry import Point, Point2D, Line, intersection

class Drone:
    def __init__(self, origin, drone_data):
        self.id = drone_data['id']
        self.flightplan = drone_data['flightplan']
        self.hostname = drone_data['hostname']
        self.port = drone_data['port']
        self.origin = (origin['latitude'], origin['longitude'])
        
        print(f'Drone {self.id} instanciated')
        print(f'Drone flightplan: {self.flightplan}')

    def start(self):
        self.thread = threading.Thread(target=self.__start)
        self.thread.start()

    def join(self):
        try:
            self.thread.join()
        except: 
            pass
        finally:
            self.alive = False
            self.mqtt_client.disconnect()
            self.mqtt_thread.join()

    def __start(self):
        client = mqtt.Client()
        client.on_connect = on_connect
        client.on_message = on_message
        client.user_data_set(self)
        client.connect(self.hostname, self.port, 60)

        self.mqtt_thread = threading.Thread(target=client.loop_forever)
        self.mqtt_thread.start()
        self.mqtt_client = client

        self.alive = True
        self.target_wp = None
        self.pos_x = 0
        self.pos_y = 0
        self.pos_z = 0
        self.t_pos_x = 0
        self.t_pos_y = 0
        self.t_pos_z = 0
        self.vel_x = 0
        self.vel_y = 0
        self.vel_z = 0
        self.heading = 0
        self.state = ''
        self.time = time.time()

        t1 = threading.Thread(target=self.go)
        t1.start()
        t1.join()

    def process_cam_message(self, message):
        #print(f'Drone {self.id} received message: {json.dumps(message, indent=4)}')

        if self.id != 1: 
            return

        latitude = message['latitude']
        longitude = message['longitude']

        y = haversine(self.origin, (latitude, self.origin[1]), unit=Unit.METERS)
        x = haversine(self.origin, (self.origin[0], longitude), unit=Unit.METERS)
        if latitude < self.origin[0]:
            y = -y
        if longitude < self.origin[1]:
            x = -x
            
        stationID = message['stationID']
        heading = message['heading']
        altitude = message['altitude']
        speed = message['speed']

        print(f'Drone {self.id} knows Drone {stationID} is at {(x, y)} heading {heading}')

        my_line = Line(Point(self.pos_x, self.pos_y), Point(self.pos_x + sin(self.heading), self.pos_y + cos(self.heading)))
        other_line = Line(Point(x, y), Point(x + sin(heading), y + cos(heading)))     

        if sqrt((self.pos_x - x)**2 + (self.pos_y - y)**2) > 500:
            return
        
        if abs(self.pos_z - altitude) > 6:
            return

        intersection = my_line.intersection(other_line)
        if len(intersection) > 0:   
            collision_point = intersection[0].evalf()
            if (self.vel_x > 0 and collision_point.x < self.pos_x) or (self.vel_x < 0 and collision_point.x > self.pos_x):
                return
            
            if (self.vel_y > 0 and collision_point.y < self.pos_y) or (self.vel_y < 0 and collision_point.y > self.pos_y):
                return

            my_time = sqrt((self.pos_x - collision_point.x)**2 + (self.pos_y - collision_point.y)**2) / sqrt(self.vel_x**2 + self.vel_y**2)
            other_time = sqrt((x - collision_point.x)**2 + (y - collision_point.y)**2) / speed

            print(f'Drone {self.id} detected a probable collision with {stationID} at: {collision_point} in {my_time}:{other_time} seconds')

    def process_denm_message(self, message):
        pass     

    def go(self):
        
        max_horizontal_velocity = self.flightplan["max_horizontal_velocity"]
        max_horizontal_acceleration = self.flightplan["max_horizontal_acceleration"]
        max_vertical_velocity = self.flightplan["max_vertical_velocity"]
        max_vertical_acceleration = self.flightplan["max_vertical_acceleration"]
        landing_rod = self.flightplan["landing_rod"]

        pos_x = 0
        pos_y = 0
        pos_z = 0
        vel_x = 0
        vel_y = 0
        vel_z = 0
        state = ''
        last_update = time.time()

        positions = []
        velocities = []

        for i in range(len(self.flightplan['waypoints'])):
            waypoint = self.flightplan['waypoints'][i]

            print(f'Waypoint {i}: {waypoint}')

            if i == 0:
                pos_x = waypoint['longitude']
                pos_y = waypoint['latitude']
                pos_z = waypoint['altitude']
                state = 'takeoff'
                vel_x = 0
                vel_y = 0
                vel_z = 0
                continue

            t_pos_x = waypoint['longitude']
            t_pos_y = waypoint['latitude']
            t_pos_z = waypoint['altitude']

            x_mov = t_pos_x > pos_x
            y_mov = t_pos_y > pos_y
            z_mov = t_pos_z > pos_z

            if i == len(self.flightplan['waypoints'])-1:
                state = 'landing-prep'

            alive = True
            while alive:
                current_time = time.time()
                time_delta = current_time - last_update
                last_update = current_time
                
                heading = atan2(t_pos_x - pos_x, t_pos_y - pos_y)
                if heading < 0:
                    heading += 2 * pi

                current_velocity = sqrt( vel_x**2 + vel_y**2 )
                current_velocity = current_velocity + max_horizontal_acceleration * time_delta  
                if current_velocity > max_horizontal_velocity:
                    current_velocity = max_horizontal_velocity

                vel_x = current_velocity * sin(heading)  
                vel_y = current_velocity * cos(heading)
                
                if state == 'takeoff':
                    vel_z = vel_z + max_vertical_acceleration * time_delta
                    if vel_z > max_vertical_velocity:
                        vel_z = max_vertical_velocity
                    
                    pos_z = pos_z + vel_z * time_delta

                    if pos_z >= t_pos_z:
                        pos_z = t_pos_z
                        vel_z = 0
                        state = 'cruise'

                elif state == 'landing-prep':
                    dest_distance = sqrt((t_pos_x - pos_x)**2 + (t_pos_y - pos_y)**2)
                    time_to_dest = dest_distance / current_velocity
                    if (pos_z - t_pos_z) / landing_rod >= time_to_dest:
                        state = 'landing'

                elif state == 'landing':
                    vel_z = -landing_rod
                    pos_z += vel_z * time_delta
                    if pos_z < t_pos_z:
                        pos_z = t_pos_z

                elif state == 'cruise':
                    if z_mov:
                        vel_z += max_vertical_acceleration * time_delta
                        if vel_z > max_vertical_velocity:
                            vel_z = max_vertical_velocity
                        pos_z += vel_z * time_delta
                        if pos_z >= t_pos_z:
                            pos_z = t_pos_z
                            vel_z = 0
                    else:
                        vel_z -= max_vertical_acceleration * time_delta
                        if vel_z < -max_vertical_velocity:
                            vel_z = -max_vertical_velocity
                        pos_z += vel_z * time_delta
                        if pos_z <= t_pos_z:
                            pos_z = t_pos_z
                            vel_z = 0

                x_arrived = False
                y_arrived = False
                z_arrived = False

                if x_mov:
                    if pos_x >= t_pos_x:
                        x_arrived = True
                        if state == 'landing-prep':
                            vel_x = 0
                else:
                    if pos_x <= t_pos_x:
                        x_arrived = True
                        if state == 'landing-prep':
                            vel_x = 0

                if y_mov:
                    if pos_y >= t_pos_y:
                        y_arrived = True
                        if state == 'landing-prep':
                            vel_y = 0
                else:
                    if pos_y <= t_pos_y:
                        y_arrived = True
                        if state == 'landing-prep':
                            vel_y = 0

                if z_mov:
                    if pos_z >= t_pos_z:
                        z_arrived = True
                        if state == 'landing-prep':
                            vel_z = 0
                else:
                    if pos_z <= t_pos_z:
                        z_arrived = True
                        if state == 'landing-prep':
                            vel_z = 0

                if not x_arrived:
                    pos_x += vel_x * time_delta

                if not y_arrived:
                    pos_y += vel_y * time_delta

                if x_arrived and y_arrived and (z_arrived or state != 'landing'):
                    # pos_x = t_pos_x
                    # pos_y = t_pos_y
                    # pos_z = t_pos_z
                    alive = False

                positions.append({'x': pos_x, 'y': pos_y, 'z': pos_z})
                velocities.append({'x': vel_x, 'y': vel_y, 'z': vel_y})
                self.heading = heading
                self.pos_x = pos_x
                self.pos_y = pos_y
                self.pos_z = pos_z
                self.vel_x = vel_x
                self.vel_y = vel_y
                self.vel_z = vel_z
                self.state = state
                self.generate_cam()

                #print(f"x: {pos_x} y: {pos_y} z: {pos_z} vx: {vel_x} vy: {vel_y} vz: {vel_z}")
                time.sleep(0.5)
        
        with open(f'./drone_{self.id}_pos.json', 'w') as outfile:
            outfile.write(json.dumps({'positions': positions}, indent= 4))
        
        with open(f'./drone_{self.id}_vel.json', 'w') as outfile:
            outfile.write(json.dumps({'velocities': velocities}, indent= 4))

        self.alive = False


    def generate_cam(self):
        with open('../examples/in_cam.json', 'r') as f:
            m = json.load(f)

            heading_origin = atan2(self.pos_x, self.pos_y)
            if heading_origin < 0:
                heading_origin += 2 * pi

            latitude, longitude = inverse_haversine(self.origin, sqrt(self.pos_x**2 + self.pos_y**2), heading_origin, unit=Unit.METERS)

            m["altitude"] = self.pos_z
            m["longitude"] = longitude
            m["latitude"] = latitude
            m["heading"] = self.heading
            m["speed"] = sqrt(self.vel_x**2 + self.vel_y**2)
            m["speedLimiter"] = m["speed"] == self.flightplan["max_horizontal_velocity"]
            m["cruiseControl"] = True
            m["driveDirection"] = "FORWARD"
            m["stationID"] = self.id
            m["stationType"] = 255

            m = json.dumps(m)
            self.mqtt_client.publish("vanetza/in/cam",m)

    def generate_denm(self):
        with open('../examples/in_denm.json', 'r') as f:
            m = json.load(f)
            
            m = json.dumps(m)
            self.mqtt_client.publish("vanetza/in/denm",m)    

def on_connect(client, userdata, flags, rc):
    print("Connected with result code " + str(rc))
    client.subscribe("vanetza/out/cam")
    client.subscribe("vanetza/out/denm")

def on_message(client: mqtt.Client, userdata: Drone, msg: mqtt.MQTTMessage):
    message = json.loads(msg.payload.decode('utf-8'))
    
    if msg.topic == 'vanetza/out/cam':
        userdata.process_cam_message(message)
    if msg.topic == 'vanetza/out/denm':
        userdata.process_denm_message(message)
