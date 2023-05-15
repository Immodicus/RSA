import json
import time
import paho.mqtt.client as mqtt

from drone_action import Action
from math import sqrt, atan2, pi, sin, cos, radians, degrees
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
        self.coll_points: list = []
        self.coll_avd_active: bool = False
        self.coll_avd_point: Point3D = None
        self.coll_avd_action: Action = None
        self.coll_avd_action_value = None
        self.coll_denm_seq: int = 0
        self.coll_denm_awareness: dict = {}

        t1: Thread = Thread(target=self.go)
        t1.start()
        t1.join()

    def init_live_server_conn(self, host: int, port: int):
        print(f'Drone {self.id} connecting to live server at {host}:{port}')
        self.live_server: socket = socket(AF_INET, SOCK_STREAM)
        self.live_server.connect((host, port))
        print(f'Drone {self.id} connected to live server')

    def live_server_send(self):
        collision_points: list = []
        for coll_point in [c['point'] for c in self.coll_points]:
            collision_points.append({
                            'latitude': float(coll_point.x), 
                            'longitude': float(coll_point.y), 
                            'altitude': float(coll_point.z)
                        })
        
        data = {
                'drone_id': self.id, 
                'latitude': self.latitude, 
                'longitude': self.longitude, 
                'altitude': self.pos_z,
                'heading': self.heading, 
                'horizontal_velocity': sqrt(self.vel_x**2 + self.vel_y**2),
                'probable_collision_points': collision_points
                }

        json_text = json.dumps(data, indent=4)
        self.live_server.sendall(bytes(json_text,encoding="utf-8"))

    def process_cam_message(self, message):

        # get relevant CAM message fields
        cam_stationID = message['stationID']
        cam_latitude = message['latitude']
        cam_longitude = message['longitude']
        cam_heading = radians(message['heading'])
        cam_altitude = message['altitude']
        cam_speed = message['speed']

        current_time = time.time()

        # process CAM sender position from latitude and longitude
        y, x = self.get_position_from_lat_lon(cam_latitude, cam_longitude)

        print(f'Drone {self.id} knows Drone {cam_stationID} is at {(x, y, cam_altitude)} heading {cam_heading}')

        # update CAM sender position or remove if stale
        if cam_stationID not in self.cam_awareness:
            print(f'Drone {self.id} is now aware of drone {cam_stationID}')
            if self.coll_avd_active:
                closest_point = self.coll_avd_point
                collision_latitude, collision_longitude = self.get_lat_lon_from_position(float(closest_point.x), float(closest_point.y))
                self.generate_denm(Point3D(collision_longitude, collision_latitude, closest_point.z), [Point3D(collision_longitude, collision_latitude, self.coll_avd_action_value)])
        
        self.cam_awareness[cam_stationID] = {
            'received_at': current_time, 
            'x': x,
            'y': y,
            'z': cam_altitude,
            'h_speed': cam_speed,
            'heading': cam_heading,
            }
        
        stales = [stationID for stationID, awareness in self.cam_awareness.items() if current_time - awareness['received_at'] > self.cam_stale_time]
        
        for stale in stales:
            del self.cam_awareness[stale]
            print(f'Drone {self.id} removed stale entry for drone {stale}')

        self.collision_cleanup()
        self.collision_detection()

    # replace me by removing the point when collision avoidance is disabled
    def collision_cleanup(self):
        stale = []
        for coll_point in [c for c in self.coll_points if not c['stale']]:
            collision_point = coll_point['point']
            if ((self.vel_x > 0 and collision_point.x < self.pos_x) or (self.vel_x < 0 and collision_point.x > self.pos_x)) or (
                (self.vel_y > 0 and collision_point.y < self.pos_y) or (self.vel_y < 0 and collision_point.y > self.pos_y)
               ):
                stale.append(coll_point)
        
        for s in stale:
            index = self.coll_points.index(s)
            self.coll_points[index]['stale'] = True
            print(f'Drone {self.id} set stale collision point: {s}')

    def collision_detection(self):
        # predict my future path based on my heading and position
        my_line = Line(Point(self.pos_x, self.pos_y), Point(self.pos_x + sin(self.heading), self.pos_y + cos(self.heading)))

        for cam_stationID, cam_data in self.cam_awareness.items():
            cam_heading = cam_data['heading']
            cam_x = cam_data['x']
            cam_y = cam_data['y']
            cam_z = cam_data['z']
            cam_speed = cam_data['h_speed']
            
            # predict the future path of this drone based on its heading and position
            cam_line = Line(Point(cam_x, cam_y), Point(cam_x + sin(cam_heading), cam_y + cos(cam_heading)))     

            # if we're to far away there's no point in trying to detect a collision given possible future 
            # heading changes
            if sqrt((self.pos_x - cam_x)**2 + (self.pos_y - cam_y)**2) > self.radio_range:
                return
            
            # if the altitude delta is below the defined threshold we're fine
            if abs(self.pos_z - cam_z) > self.min_safe_altitude_delta:
                return

            # calculate 2D intersection between our lines
            intersection: list[Point2D] = my_line.intersection(cam_line)
            if len(intersection) > 0:   
                collision_point = intersection[0].evalf()

                if (self.vel_x > 0 and collision_point.x < self.pos_x) or (self.vel_x < 0 and collision_point.x > self.pos_x):
                    continue
                
                if (self.vel_y > 0 and collision_point.y < self.pos_y) or (self.vel_y < 0 and collision_point.y > self.pos_y):
                    continue

                # calculate time to the collision point for me and the other drone
                my_time = sqrt((self.pos_x - collision_point.x)**2 + (self.pos_y - collision_point.y)**2) / sqrt(self.vel_x**2 + self.vel_y**2)
                cam_time = sqrt((cam_x - collision_point.x)**2 + (cam_y - collision_point.y)**2) / cam_speed

                # if we arrive at the collision point at very different times we're fine
                if abs(my_time - cam_time) > 3:
                    continue

                # calculate average altitude between our altitudes
                collision_altitude = round((self.pos_z + cam_z) / 2)
                collision_point = Point3D(collision_point.x, collision_point.y, collision_altitude).evalf()

                # determine if this collision point is already known
                found = False
                for point in [c['point'] for c in self.coll_points]:
                    if abs(float(point.x) - collision_point.x) < 5 and abs(float(point.x) - collision_point.x) < 5:
                        found = True
                
                # if we weren't aware of it, now we are
                if not found:
                    print(f'Drone {self.id} detected a probable collision with {cam_stationID} at: {collision_point} in {my_time}:{cam_time} seconds')
                    self.coll_points.append({'point': collision_point, 'stale': False})

                    if not self.coll_avd_active:
                        self.make_a_decision()

    def make_a_decision(self):
        # determine the closest collision point known to us
        closest_point_distance = 2**64-1
        closest_point = None
        for collision_point in [c['point'] for c in self.coll_points if not c['stale']]:
            distance = sqrt((self.pos_x - collision_point.x)**2 + (self.pos_y - collision_point.y)**2)
            if distance < closest_point_distance:
                closest_point_distance = distance
                closest_point = collision_point

        assert closest_point != None     
        assert self.coll_avd_active == False
        
        # update collision avoidance status
        self.coll_avd_active = True
        self.coll_avd_point = closest_point
        
        if self.pos_z >= float(closest_point.z):
            self.coll_avd_action = Action.ALT_INCR
            self.coll_avd_action_value = float(closest_point.z) + self.min_safe_altitude_delta / 2 + 0.5
        else:
            self.coll_avd_action = Action.ALT_DEC
            self.coll_avd_action_value = float(closest_point.z) - self.min_safe_altitude_delta / 2 - 0.5
            
        # send a denm message
        collision_latitude, collision_longitude = self.get_lat_lon_from_position(float(closest_point.x), float(closest_point.y))
        self.generate_denm(Point3D(collision_longitude, collision_latitude, closest_point.z), [Point3D(collision_longitude, collision_latitude, self.coll_avd_action_value)])
        
        print(f'Drone {self.id} sending denm {closest_point.x, closest_point.y, self.coll_avd_action_value}')

    def update_decision(self):
        print(f'Drone {self.id} update_decision')
        if not self.coll_avd_active:
            return
        
        restrictions = []

        # find denm messages from other drones regarding the active collision point
        for denm_stationID, denm_data in self.coll_denm_awareness.items():
            event_x = denm_data['event_x']
            event_y = denm_data['event_y']
            event_z = denm_data['event_z']
            decision_z = denm_data['decision_z']

            if abs(self.coll_avd_point.x - event_x) < 5 and abs(self.coll_avd_point.y - event_y) < 5:
                if denm_stationID > self.id:
                    restrictions.append((denm_stationID, decision_z, event_z))

        print(f'Drone {self.id} restrictions: {restrictions}')

        # we're good
        if len(restrictions) == 0:
            return
        
        # we might need to change our decision
        # if there's only one restriction and it's the same as us do the opposite
        changed = False
        if len(restrictions) == 1:
            print(f'Drone: {self.id} {self.coll_avd_point}')
            decision_z = restrictions[0][1]
            event_z = restrictions[0][2]
            if self.coll_avd_action == Action.ALT_INCR and decision_z > self.coll_avd_point.z:
                self.coll_avd_action = Action.ALT_DEC
                self.coll_avd_action_value = event_z - self.min_safe_altitude_delta / 2 - 0.5
                
                changed = True
            elif self.coll_avd_action == Action.ALT_DEC and decision_z < self.coll_avd_point.z:
                self.coll_avd_action = Action.ALT_INCR
                self.coll_avd_action_value = event_z + self.min_safe_altitude_delta / 2 + 0.5

                changed = True

        # generate new DENM
        if changed:
            print(f'Drone {self.id} changed decision to {self.coll_avd_action}: {self.coll_avd_action_value}')
            collision_latitude, collision_longitude = self.get_lat_lon_from_position(self.coll_avd_point.x, self.coll_avd_point.y)
            self.generate_denm(Point3D(collision_longitude, collision_latitude, self.coll_avd_point.z), [Point3D(collision_longitude, collision_latitude, self.coll_avd_action_value)])

    def generate_denm(self, collision_point: Point3D, avoidance_path: list[Point3D]):
        collision_point = collision_point.evalf()
        avoidance_path = [point.evalf() for point in avoidance_path]
        
        with open('messages/in_denm.json', 'r') as f:
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

    def process_denm_message(self, message):
        message = message['fields']['denm']
        denm_stationID = message['management']['actionID']['originatingStationID']

        if message['situation']['eventType']['causeCode'] != 97:
            return
        
        event_position = message['management']['eventPosition']
        denm_collision_point: Point2D = Point2D(event_position['longitude'], event_position['latitude']).evalf()

        print(f'Drone {self.id} received denm from {denm_stationID} at {denm_collision_point}')
        
        denm_sequence_number = message['management']['actionID']['sequenceNumber']

        event_latitude = event_position['latitude']
        event_longitude = event_position['longitude']
        event_altitude = event_position['altitude']['altitudeValue']

        #read and process event positioning and decision data
        event_y, event_x = self.get_position_from_lat_lon(event_latitude, event_longitude)
        event_z = event_position['altitude']['altitudeValue']

        decision_point = message['alacarte']['roadWorks']['recommendedPath'][0]
        decision_y, decision_x = self.get_position_from_lat_lon(decision_point['latitude'], decision_point['longitude'])
        decision_z = decision_point['altitude']['altitudeValue']
        
        # update denm awareness
        self.coll_denm_awareness[denm_stationID] = {
            'sequence_number': denm_sequence_number,
            'event_x': event_x,
            'event_y': event_y,
            'event_z': event_z,
            'decision_x': decision_x,
            'decision_y': decision_y,
            'decision_z': decision_z
            }
        
        self.update_decision()

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

                # if collision avoidance is active override settings
                # only target altitude for now
                if self.coll_avd_active == True:
                    t_pos_z = self.coll_avd_action_value
                    z_mov = t_pos_z > pos_z

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

                if self.coll_avd_active == True:
                    # if we're past the collision point, disable collision avoidance and remove restricitons
                    if ((vel_x > 0 and self.coll_avd_point.x < pos_x) or (vel_x < 0 and self.coll_avd_point.x > pos_x)) and (
                        (vel_y > 0 and self.coll_avd_point.y < pos_y) or (vel_y < 0 and self.coll_avd_point.y > pos_y)
                        ):
                        print(f'Drone {self.id} disabling collision avoidance')
                        self.coll_avd_active = False
                        t_pos_x = waypoint['longitude']
                        t_pos_y = waypoint['latitude']
                        t_pos_z = waypoint['altitude']
                        x_mov = t_pos_x > pos_x
                        y_mov = t_pos_y > pos_y
                        z_mov = t_pos_z > pos_z

                latitude, longitude = self.get_lat_lon_from_position(pos_x, pos_y)

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

                time.sleep(0.5)
        
        with open(f'./drone_{self.id}_pos.json', 'w') as outfile:
            outfile.write(json.dumps({'positions': positions}, indent= 4))
        
        with open(f'./drone_{self.id}_vel.json', 'w') as outfile:
            outfile.write(json.dumps({'velocities': velocities}, indent= 4))

        with open(f'./drone_{self.id}_coords.json', 'w') as outfile:
            outfile.write(json.dumps({'coordinates': coordinates}, indent= 4))

        self.alive = False


    def generate_cam(self):
        with open('messages/in_cam.json', 'r') as f:
            m = json.load(f)

            m["altitude"] = self.pos_z
            m["longitude"] = self.longitude
            m["latitude"] = self.latitude
            m["heading"] = degrees(self.heading)
            m["speed"] = sqrt(self.vel_x**2 + self.vel_y**2)
            m["speedLimiter"] = m["speed"] == self.flightplan["max_horizontal_velocity"]
            m["cruiseControl"] = True
            m["driveDirection"] = "FORWARD"
            m["stationID"] = self.id
            m["stationType"] = 255

            m = json.dumps(m)
            self.mqtt_client.publish("vanetza/in/cam",m)  

    def get_lat_lon_from_position(self, pos_x: float, pos_y: float):
        heading_origin = get_heading_from_origin(pos_x, pos_y)
        return inverse_haversine(self.origin, sqrt(pos_x**2 + pos_y**2), heading_origin, unit=Unit.METERS)
    
    def get_position_from_lat_lon(self, lat: float, lon: float):
        y = haversine(self.origin, (lat, self.origin[1]), unit=Unit.METERS)
        x = haversine(self.origin, (self.origin[0], lon), unit=Unit.METERS)
        if lat < self.origin[0]:
            y = -y
        if lon < self.origin[1]:
            x = -x

        return y, x

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
