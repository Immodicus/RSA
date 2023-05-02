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

## Running

1. Change ***simulation.json*** file to match your preferences and adjust ***docker-compose.yml*** appropriately

2. Install the required dependencies
```bash
pip3 install -r requirements.txt
```

3. Run the simulation
```bash
python3 main.py -simulation_file simulation.json
```

## Running Frontend
Get in into frontend folder and run the following commands
```
python3 -m venv venv
```
```
source venv/bin/activate
```
```
pip3 install -r requirements.txt
```
```
flask --app map run or python3 map.py
```
