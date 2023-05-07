import socket
import _thread
import sys

def client_echo_thread(client_socket: socket.socket, addr):
    while True:
        message = client_socket.recv(1024)
        if not message: break
        
        message_text = message.decode("utf-8")
        print(f"Received {message_text} from {addr}")
    
    print(f"Closing connection to {addr}")
    client_socket.close()

server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
server_socket.bind(('', int(sys.argv[1])))
server_socket.listen()
print(f"Starting TCP echo server at port {int(sys.argv[1])}")

while True:
    client_socket, addr = server_socket.accept()
    print(f"Accepted new client at: {addr}")
    _thread.start_new_thread(client_echo_thread, (client_socket, addr))

server_socket.close()