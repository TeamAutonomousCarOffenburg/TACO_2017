import socket
import cv2
import numpy as np
UDP_IP = "127.0.0.1"
UDP_PORT = 1337

if __name__ == '__main__':
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.bind((UDP_IP, UDP_PORT))
    sock.listen(1)
    print("listening...")
    stream = cv2.VideoCapture(0)
    (grabbed, frame) = stream.read()
    height, width, channels = frame.shape
    print("h: {} w: {} ch: {}".format(height, width, channels))
    while True:
        conn, addr = sock.accept()
        print("client connected")
        while True:
            (grabbed, frame) = stream.read()
            try:
                bytes = conn.send(frame)
            except Exception as e:
                conn.close()
                print("error on send")
                cv2.destroyAllWindows()
                break

            cv2.imshow('my webcam', frame)
            if cv2.waitKey(1) == 27:
                break  # esc to quit

        conn.close()
        print("client disconnected")
    sock.close()
