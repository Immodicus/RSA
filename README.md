# RSA - ADCAS

## Setup

1. Clone the NAP-Vanetza repository
```bash
git clone https://code.nap.av.it.pt/mobility-networks/vanetza.git
cd vanetza
```

2. Change the periodicity of CAM messages in ***tools/socktap/config.ini*** to `0`
```
[cam]
...                             
periodicity=0                                ; in milliseconds - 0 to disable
...
```

3. Add the following volume to every container in ***docker-compose.yml***
```
volumes:
    - ./tools/socktap/config.ini:/config.ini
```

4. Create the following docker network and start the containers
```bash
docker network create vanetzalan0 --subnet 192.168.98.0/24

docker compose up
```

## Running the simulation seperately

1. Change ***simulation.json*** file to match your preferences (or use one of the included files in the ***/simulation*** directory)

2. Adjust the ***docker-compose.yml*** file in the vanetza root directory appropriately (or copy the included one from the ***/simulation*** directory)

3. Install the required dependencies
```bash
cd simulation
pip3 install -r requirements.txt
```

4. Start the tcp server (this is required even if you're not running the frontend)
```bash
cd frontend
python3 simulate_live.py 6000 # this number must match the live_server_port defined in the simulation file you're using
```

5. Run the simulation in another terminal window
```bash
cd simulation
python3 main.py -simulation_file simulation.json
```

## Running the simulation from the frontend interface
Get in into frontend folder and run the following commands

1. Install the required dependencies
```bash
python3 -m venv venv
source venv/bin/activate
pip3 install -r requirements.txt
```
2. Do step `1` through `4` of the [previous section](#running-the-simulation-seperately)

3. Start the frontend interface
```bash
flask --app map run
# or
python3 map.py
```
