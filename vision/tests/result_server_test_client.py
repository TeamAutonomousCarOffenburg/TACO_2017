import socket

SERVER_IP = "127.0.0.1"
SERVER_PORT = 63237
LENGTH = 2048

if __name__ == '__main__':

    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.connect((SERVER_IP, SERVER_PORT))
    print("connected to server")

    while True:
        chunk = sock.recv(LENGTH)
        if not chunk:
            print("error on receive")
            break

        print("data: {}".format(chunk[4:].decode()))

    sock.shutdown(socket.SHUT_RDWR)
    sock.close()
