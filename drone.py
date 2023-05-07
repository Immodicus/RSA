import json
import time
import paho.mqtt.client as mqtt

from math import sqrt, atan2, pi, sin, cos
from haversine import haversine, Unit, inverse_haversine
from sympy.geometry import Point, Point2D, Line, Point3D
from threading import Thread
from socket import socket, AF_INET, SOCK_STREAM

class Drone:
    def __init__(self, settings, drone_data):
        self.id = drone_data['id']
        self.flightplan = drone_data['flightplan']
        self.hostname = drone_data['hostname']
        self.port = drone_data['port']
        self.origin = (settings['origin']['latitude'], settings['origin']['longitude'])
        self.radio_range = settings['radio_range']
        self.min_safe_altitude_delta = settings['min_safe_altitude_delta']
        self.cam_stale_time = settings['cam_stale_time']

        self.init_live_server_conn(settings['live_server_address'], settings['live_server_port'])
        
        print(f'Drone {self.id} instanciated')
        print(f'Drone flightplan: {self.flightplan}')

    def start(self):
        self.thread: Thread = Thread(target=self.__start)
        self.thread.start()

    def join(self):
        try:
            self.thread.join()
        except: 
            pass
        finally:
            self.alive = False
            self.live_server.close()
            self.mqtt_client.disconnect()
            self.mqtt_thread.join()

    def __start(self):
        client: mqtt.Client = mqtt.Client()
        client.on_connect = on_connect
        client.on_message = on_message
        client.user_data_set(self)
        client.connect(self.hostname, self.port, 60)

        self.mqtt_thread: Thread = Thread(target=client.loop_forever)
        self.mqtt_thread.start()
        self.mqtt_client = client

        self.alive: bool = True
        self.pos_x: float = 0
        self.pos_y: float = 0
        self.pos_z: float = 0
        self.t_pos_x: float = 0
        self.t_pos_y: float = 0
        self.t_pos_z: float = 0
        self.vel_x: float = 0
        self.vel_y: float = 0
        self.vel_z: float = 0
        self.heading: float = 0
        self.state: str = ''
        self.latitude: float = 0
        self.longitude: float = 0

        self.time: float = time.time()

        self.cam_awareness: dict = {}

        self.coll_avd_path = None
        self.coll_avd_active: bool = False
        self.coll_point: Point3D = None
        self.coll_denm_seq: int = 0

        t1: Thread = Thread(target=self.go)
        t1.start()
        t1.join()

    def init_live_server_conn(self, host: int, port: int):
        print(f'Drone {self.id} connecting to live server at {host}:{port}')
        self.live_server: socket = socket(AF_INET, SOCK_STREAM)
        self.live_server.connect((host, port))
        print(f'Drone {self.id} connected to live server')

    def live_server_send(self):
        collision_point = None
        if self.coll_point != None:
            collision_point = {
                            'latitude': float(self.coll_point.y), 
                            'longitude': float(self.coll_point.x), 
                            'altitude': float(self.coll_point.x)
                        }
        
        data = {
                    'drone_id': self.id, 
                    'latitude': self.latitude, 
                    'longitude': self.longitude, 
                    'heading': self.heading, 
                    'horizontal_velocity': sqrt(self.vel_x**2 + self.vel_y**2),
                    'probable_collision_points': [
                        collision_point
                    ]
                }

        json_text = json.dumps(data, indent=4)
        self.live_server.sendall(bytes(json_text,encoding="utf-8"))

    def process_cam_message(self, message):
        #print(f'Drone {self.id} received message: {json.dumps(message, indent=4)}')

        cam_stationID = message['stationID']
        cam_latitude = message['latitude']
        cam_longitude = message['longitude']
        cam_heading = message['heading']
        cam_altitude = message['altitude']
        cam_speed = message['speed']

        self.awareness_update(cam_stationID)

        y = haversine(self.origin, (cam_latitude, self.origin[1]), unit=Unit.METERS)
        x = haversine(self.origin, (self.origin[0], cam_longitude), unit=Unit.METERS)
        if cam_latitude < self.origin[0]:
            y = -y
        if cam_longitude < self.origin[1]:
            x = -x

        print(f'Drone {self.id} knows Drone {cam_stationID} is at {(x, y)} heading {cam_heading}')

        my_line = Line(Point(self.pos_x, self.pos_y), Point(self.pos_x + sin(self.heading), self.pos_y + cos(self.heading)))
        cam_line = Line(Point(x, y), Point(x + sin(cam_heading), y + cos(cam_heading)))     

        if sqrt((self.pos_x - x)**2 + (self.pos_y - y)**2) > self.radio_range:
            return
        
        if abs(self.pos_z - cam_altitude) > self.min_safe_altitude_delta:
            return

        intersection: list[Point2D] = my_line.intersection(cam_line)
        if len(intersection) > 0:   
            collision_point = intersection[0].evalf()
            
            heading_origin = get_heading_from_origin(collision_point.x, collision_point.y)
            collision_latitude, collision_longitude = inverse_haversine(self.origin, sqrt(collision_point.x**2 + collision_point.y**2), heading_origin, unit=Unit.METERS)

            if (self.vel_x > 0 and collision_point.x < self.pos_x) or (self.vel_x < 0 and collision_point.x > self.pos_x):
                return
            
            if (self.vel_y > 0 and collision_point.y < self.pos_y) or (self.vel_y < 0 and collision_point.y > self.pos_y):
                return

            my_time = sqrt((self.pos_x - collision_point.x)**2 + (self.pos_y - collision_point.y)**2) / sqrt(self.vel_x**2 + self.vel_y**2)
            cam_time = sqrt((x - collision_point.x)**2 + (y - collision_point.y)**2) / cam_speed

            if not self.coll_avd_active:
                print(f'Drone {self.id} detected a probable collision with {cam_stationID} at: {collision_point} in {my_time}:{cam_time} seconds')
                self.coll_avd_active = True
                self.coll_point = Point3D(collision_latitude, collision_longitude, cam_altitude).evalf()
                
                decision_altitude = None
                if self.id > cam_stationID:
                    decision_altitude = self.pos_z + 5
                else:
                    decision_altitude = self.pos_z - 5

                self.generate_denm(Point3D(collision_latitude, collision_longitude, cam_altitude), [Point3D(collision_latitude, collision_longitude, decision_altitude)])

    def generate_denm(self, collision_point: Point3D, avoidance_path: list[Point3D]):
        collision_point = collision_point.evalf()
        avoidance_path = [point.evalf() for point in avoidance_path]
        
        with open('../examples/in_denm.json', 'r') as f:
            m = json.load(f)
            
            m['management']['actionID']['originatingStationID'] = self.id
            m['management']['actionID']['sequenceNumber'] = self.coll_denm_seq
            m['management']['detectionTime'] = time.time()
            m['management']['referenceTime'] = time.time()
            m['management']['eventPosition']['latitude'] = float(collision_point.y)
            m['management']['eventPosition']['longitude'] = float(collision_point.x)
            m['management']['eventPosition']['altitude']['altitudeValue'] = float(collision_point.z)
            m['management']['stationType'] = 255

            m['situation']['eventType']['causeCode'] = 97
            m['situation']['eventType']['subCauseCode'] = 97

            m['alacarte']['roadWorks']['recommendedPath'] = []
            for point in avoidance_path:
                m['alacarte']['roadWorks']['recommendedPath'].append(
                    {
                        'latitude': float(point.y), 'longitude': float(point.x), 'altitude': {'altitudeValue': float(point.z), 'altitudeConfidence': 1},
                        'positionConfidenceEllipse': {
                            'semiMajorConfidence': 0,
                            'semiMinorConfidence': 0,
                            'semiMajorOrientation': 0
                        }
                    }
                    )

            m = json.dumps(m)
            
            self.mqtt_client.publish("vanetza/in/denm",m)  
            self.coll_denm_seq += 1

    def awareness_update(self, stationID: int):       
        current_time = time.time()
        
        if stationID not in self.cam_awareness:
            print(f'Drone {self.id} is now aware of drone {stationID}')
        self.cam_awareness[stationID] = time.time()
        
        stales = [stationID for stationID, lastCam in self.cam_awareness.items() if current_time - lastCam > self.cam_stale_time]
        
        for stale in stales:
            del self.cam_awareness[stale]
            print(f'Drone {self.id} removed stale entry for drone {stale}')

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
        coordinates = []

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
                
                heading = get_heading_between_points(pos_x, t_pos_x, pos_y, t_pos_y)

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
                    alive = False

                heading_origin = get_heading_from_origin(pos_x, pos_y)
                latitude, longitude = inverse_haversine(self.origin, sqrt(pos_x**2 + pos_y**2), heading_origin, unit=Unit.METERS)

                positions.append({'x': pos_x, 'y': pos_y, 'z': pos_z})
                velocities.append({'x': vel_x, 'y': vel_y, 'z': vel_y})
                coordinates.append({'latitude': latitude, 'longitude': longitude})
                
                self.heading = heading
                self.pos_x = pos_x
                self.pos_y = pos_y
                self.pos_z = pos_z
                self.vel_x = vel_x
                self.vel_y = vel_y
                self.vel_z = vel_z
                self.state = state
                self.latitude = latitude
                self.longitude = longitude
                self.generate_cam()
                self.live_server_send()

                #print(f"x: {pos_x} y: {pos_y} z: {pos_z} vx: {vel_x} vy: {vel_y} vz: {vel_z}")
                time.sleep(0.5)
        
        with open(f'./drone_{self.id}_pos.json', 'w') as outfile:
            outfile.write(json.dumps({'positions': positions}, indent= 4))
        
        with open(f'./drone_{self.id}_vel.json', 'w') as outfile:
            outfile.write(json.dumps({'velocities': velocities}, indent= 4))

        with open(f'./drone_{self.id}_coords.json', 'w') as outfile:
            outfile.write(json.dumps({'coordinates': coordinates}, indent= 4))

        self.alive = False


    def generate_cam(self):
        with open('../examples/in_cam.json', 'r') as f:
            m = json.load(f)

            m["altitude"] = self.pos_z
            m["longitude"] = self.longitude
            m["latitude"] = self.latitude
            m["heading"] = self.heading
            m["speed"] = sqrt(self.vel_x**2 + self.vel_y**2)
            m["speedLimiter"] = m["speed"] == self.flightplan["max_horizontal_velocity"]
            m["cruiseControl"] = True
            m["driveDirection"] = "FORWARD"
            m["stationID"] = self.id
            m["stationType"] = 255

            m = json.dumps(m)
            self.mqtt_client.publish("vanetza/in/cam",m)  

def get_heading_from_origin(pos_x: float, pos_y: float) -> float:
    heading_origin = atan2(pos_x, pos_y)
    if heading_origin < 0:
        heading_origin += 2 * pi

    return heading_origin

def get_heading_between_points(x_a: float, x_b: float, y_a: float, y_b: float) -> float:
    heading = atan2(x_b - x_a, y_b - y_a)
    if heading < 0:
        heading += 2 * pi 

    return heading  

def on_connect(client: mqtt.Client, userdata: Drone, flags, rc):
    print("Connected with result code " + str(rc))
    client.subscribe("vanetza/out/cam")
    client.subscribe("vanetza/out/denm")

def on_message(client: mqtt.Client, userdata: Drone, msg: mqtt.MQTTMessage):
    message = json.loads(msg.payload.decode('utf-8'))
    
    if msg.topic == 'vanetza/out/cam':
        userdata.process_cam_message(message)
    if msg.topic == 'vanetza/out/denm':
        userdata.process_denm_message(message)
