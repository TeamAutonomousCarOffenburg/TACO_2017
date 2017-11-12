import os
import time as t
from threading import Thread
import utils.tools as tools
import cv2


class VideoWriter:
    def __init__(self, im_width, im_height, folder, video_queue):
        self.video_queue = video_queue
        self.folder = folder
        self.im_width = im_width
        self.im_height = im_height
        self.writer = None
        self.file = None
        self.fps = None
        self.stopped = True
        self.thread = Thread(target=self.update, args=())

    def init(self, file_prefix="output", fps=30):
        filename = "{}_{}.avi".format(file_prefix, tools.get_timestamp_ms())
        self.file = os.path.join(self.folder, filename)
        self.fps = fps

        fourcc = cv2.VideoWriter_fourcc(*"MJPG")
        self.writer = cv2.VideoWriter(self.file, fourcc,
                                      float(self.fps), (self.im_width,
                                                        self.im_height))
        return self

    def start(self):
        self.stopped = False
        self.thread.start()
        print("[VIDEO WRITER] Thread for writing video started")
        return self

    def update(self):
        while True:
            # if self.stopped and self.video_queue.empty():
            if self.stopped:
                return
            # wait for element in queue
            try:
                image = self.video_queue.get_nowait()
            except Exception as e:
                t.sleep(0.02)
                continue

            self.writer.write(image)

    def stop(self):
        while not self.video_queue.empty():
            t.sleep(0.1)
        self.stopped = True
        self.writer.release()
        print('[VIDEO WRITER] Video written to file: {}'.format(self.file))

    def is_running(self):
        return not self.stopped

    def is_thread_alive(self):
        return self.thread.is_alive()

    def get_video_file_name(self):
        return self.file
