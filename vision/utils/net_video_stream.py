import socket
import numpy as np
from threading import Thread
import time as t


class NetVideoStream:
    def __init__(self, server_ip, server_port, width, height, channel):
        self.ip = server_ip
        self.port = server_port
        self.width = width
        self.height = height
        self.channel = channel
        self.image_size = width * height * channel
        self.sock = None
        self.connected = False
        self.stopped = True
        self.frame = False
        self.thread = None

    def connect(self):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.connected = False
        print("[STREAM] connecting to server")
        while not self.connected:
            if self.stopped:
                break
            try:
                self.sock.connect((self.ip, self.port))
                self.connected = True
                print("[STREAM] connected to server")
            except Exception as e:
                t.sleep(0.02)
                continue
        return self.connected

    def disconnect(self):
        try:
            self.sock.shutdown(socket.SHUT_RDWR)
        except Exception as e:
            print("[STREAM] error on disconnect {}".format(e))
            pass
        self.sock.close()
        self.connected = False

    def read_data(self, length):
        if self.stopped:
            return False
        msgparts = []
        while length > 0:
            try:
                chunk = self.sock.recv(length)
            except Exception as e:
                return False
            msgparts.append(chunk)
            length -= len(chunk)
        return b"".join(msgparts)

    def read_image(self):
        if self.stopped:
            return False
        length = self.image_size
        msgparts = []
        while length > 0:
            try:
                chunk = self.sock.recv(length)
                if not chunk:
                    # print(
                    #     "[STREAM] error on read data --> try to reconnect...")
                    return False
            except Exception as e:
                # print("[STREAM] error on recv data --> try to reconnect... {}".
                #       format(e))
                return False
            msgparts.append(chunk)
            length -= len(chunk)
        data = b"".join(msgparts)

        return np.fromstring(data, np.uint8).reshape(self.height, self.width,
                                                     self.channel)

    def start(self):
        self.stopped = False
        self.thread = Thread(target=self.update, args=())
        self.thread.start()
        return self

    def update(self):
        while True:
            if self.stopped:
                return

            image = self.read_image()

            if image is False:
                self.connect()
                t.sleep(0.02)
                continue

            self.frame = image
            t.sleep(0.002)

    def read(self):
        return self.frame

    def stop(self):
        self.stopped = True
        return self

    def is_connected(self):
        return self.connected

    def is_running(self):
        return not self.stopped

    def is_thread_alive(self):
        return self.thread.is_alive()
