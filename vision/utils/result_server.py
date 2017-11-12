import socket
from threading import Thread
import json
import time as t


class ResultServer:
    def __init__(self, port, queue):
        self.port = port
        self.queue = queue
        self.client_connected = False
        self.sock = None
        self.stopped = True
        self.thread = None

    def start(self):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.bind(('', self.port))
        self.sock.setblocking(0)
        self.sock.listen(1)
        self.stopped = False
        self.thread = Thread(target=self.update, args=())
        self.thread.start()
        print("[RESULT SERVER] Listening for a client")
        return self

    def update(self):
        while True:
            if self.stopped:
                return

            try:
                conn, addr = self.sock.accept()
            except Exception as e:
                t.sleep(0.02)
                continue

            print("[RESULT SERVER] Client connected")
            self.client_connected = True
            while True:
                if self.stopped:
                    self.disconnect_client(conn)
                    return

                # wait for element in queue
                try:
                    result = self.queue.get_nowait()
                except Exception as e:
                    t.sleep(0.02)
                    continue

                try:
                    bytes = conn.send(self.build_msg(result))
                    # print("[INFO] bytes send to client: {}".format(bytes))
                except Exception as e:
                    self.disconnect_client(conn)
                    print("[RESULT SERVER] client disconnected {}".format(e))
                    break
                t.sleep(0.002)

    def disconnect_client(self, conn):
        self.client_connected = False
        conn.close()

    def stop(self):
        self.stopped = True
        self.sock.close()

    def is_client_connected(self):
        return self.client_connected

    def is_running(self):
        return not self.stopped

    def is_thread_alive(self):
        return self.thread.is_alive()

    def build_msg(self, data):
        json_object = json.dumps(dict(Vision=data))
        object_size = len(json_object)
        msg = bytearray()
        msg.extend(object_size.to_bytes(4, 'little'))
        msg.extend(bytearray(json_object, 'utf8'))
        return msg
