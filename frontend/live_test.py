import socket
import _thread
import sys
import json

drone_data: dict = {}

def client_echo_thread(client_socket: socket.socket, addr):
    while True:
        message = client_socket.recv(1024)
        if not message: break
        
        message_text = message.decode("utf-8")
        message_json = json.loads(message_text)
        drone_data[message_json['drone_id']] = message_json

        #print(f"Received {message_text} from {addr}")
        t = json.dumps(list(drone_data.values()), indent=4)
        print(f'drone_data: {t}')
    
    print(f"Closing connection to {addr}")
    client_socket.close()

server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
server_socket.bind(('0.0.0.0', int(sys.argv[1])))
server_socket.listen()
print(f"Starting TCP server at port {int(sys.argv[1])}")

while True:
    client_socket, addr = server_socket.accept()
    print(f"Accepted new client at: {addr}")
    _thread.start_new_thread(client_echo_thread, (client_socket, addr))

server_socket.close()