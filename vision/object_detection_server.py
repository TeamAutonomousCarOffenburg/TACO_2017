import argparse
import math
import os
import sys
import time as t
from multiprocessing import Queue
import cv2
import signal

import utils.tools as tools
from utils import visualisation as v
from utils.fps import FPS
from utils.net_video_stream import NetVideoStream
from utils.result_server import ResultServer
from utils.tf_object_detection import ObjectDetection
from utils.video_writer import VideoWriter

args = None
stream = None
result_video_writer = None
result_server = None
fps = None


def get_args():
    parser = argparse.ArgumentParser(
        description='Process images given by ADTF Ext Python Server Module.')
    parser.add_argument(
        '-c',
        '--config',
        required=False,
        default="./config/vision_server.config.json",
        help=
        "Path to the configuration file with the settings for Object Detection"
    )
    parser.add_argument(
        '-l',
        '--logdir',
        required=False,
        default="./logs",
        help=
        "Path to the configuration file with the settings for Object Detection"
    )
    parser.add_argument('-wtl', '--write_to_logfile', action='store_true')
    parser.add_argument(
        '-ip',
        '--server-ip',
        default="127.0.0.1",
        help="Server IP from the Camera Relay")
    parser.add_argument(
        '-sp', '--server-port', type=int, default=1337, help="Port to connect")
    parser.add_argument(
        '-cp',
        '--client-port',
        type=int,
        default=63237,
        help="Port where the client connects to")
    parser.add_argument('-iw', '--image-width', type=int, default=1240)
    parser.add_argument('-ih', '--image-height', type=int, default=330)
    parser.add_argument('-ic', '--image-channels', type=int, default=3)
    parser.add_argument(
        '-v',
        '--visualize',
        action='store_true',
        help="Displays the images with the results")
    parser.add_argument('-d', '--debug', action='store_true')
    parser.add_argument('-crv', '--create-result-video', action='store_true')
    parser.add_argument('-vf', '--video-folder', default="/home/aadc/videos")
    parser.add_argument(
        '-rqs',
        '--result-queue-size',
        type=int,
        default=600,
        help=
        'Size of the result queue. Elements will be sent to the taco client')
    parser.add_argument(
        '-vqs',
        '--video-queue-size',
        type=int,
        default=600,
        help=
        'Size of the video queue. Contains Images which will be written to the video'
    )
    parser.add_argument(
        '-w',
        '--warmup',
        type=int,
        default=25,
        help='Warmup Time for Object Detection.')
    parser.add_argument(
        '-gu',
        '--gpu-usage',
        type=float,
        default=0.75,
        help="How much gpu can we use?")
    parser.add_argument(
        '-t',
        '--threshold',
        type=float,
        default=0.5,
        help='Threshold for detected objects')
    args = parser.parse_args()

    if not os.path.isfile(args.config):
        print("[ERROR] config file does not exist... {}".format(args.config))
        exit(1)

    if args.create_result_video:
        if not os.path.isdir(args.video_folder):
            print("[ERROR] folder for output vidoes does not exist... {}".
                  format(args.video_folder))
            exit(1)

    if not os.path.isdir(args.logdir):
        os.mkdir(args.logdir)

    return args


def print_fps(input, now=False):
    print('[INFO] approx. FPS| INPUT:{:.2f}'.format(input.fps(now)))
    sys.stdout.flush()


def signal_handler(sig, frame):
    stop_application()


def stop_application():
    fps.stop()
    if args.debug:
        print(
            '[INFO] elapsed time (total)| INPUT:{:.2f}'.format(fps.elapsed()))
        print_fps(fps)

    if result_video_writer is not None and result_video_writer.is_running():
        result_video_writer.stop()

    cv2.destroyAllWindows()
    result_server.stop()
    stream.stop()
    t.sleep(1)
    stream.disconnect()

    if args.debug:
        print("Stream Thread alive? {}".format(stream.is_thread_alive()))
        print("Result Server Thread alive? {}".format(
            result_server.is_thread_alive()))
        print("Bye Bye...")

    if args.write_to_logfile:
        sys.stdout.close()

    sys.exit(0)


if __name__ == '__main__':
    args = get_args()

    if args.write_to_logfile:
        logfile = "{}.log".format(tools.get_timestamp_ms())
        logfile = os.path.join(args.logdir, logfile)
        sys.stdout = open(logfile, "w")

    stream = NetVideoStream(args.server_ip, args.server_port, args.image_width,
                            args.image_height, args.image_channels)
    if not stream.start():
        print("[ERROR] could not connect to Server...")
        exit(2)

    signal.signal(signal.SIGUSR1, signal_handler)

    result_video_writer = None
    result_video_queue = None
    if args.create_result_video:
        result_video_queue = Queue(maxsize=args.video_queue_size)
        result_video_writer = VideoWriter(args.image_width, args.image_height,
                                          args.video_folder,
                                          result_video_queue)

    result_q = Queue(maxsize=args.result_queue_size)

    result_server = ResultServer(args.client_port, result_q)
    result_server.start()

    config = tools.load_config(args.config)
    od = ObjectDetection(config, args.image_width, args.image_height,
                         args.gpu_usage, args.threshold).start_up()

    dt = t.time()
    fps_dt = t.time()
    fps = FPS().start()

    while True:
        try:
            if not stream.is_connected():
                stop_application()

            image = stream.read()

            if image is False:
                t.sleep(0.02)
                continue

            result = od.detect_objects(image)

            fps.update()

            if result_server.is_client_connected():
                result_q.put(result)

            if args.visualize or args.create_result_video:
                image = v.draw_boxes_and_labels(image,
                                                result["recognizedObjects"])

            if args.create_result_video:
                if not result_video_writer.is_running(
                ) and t.time() - fps_dt > args.warmup:
                    current_fps = math.ceil(fps.fps(True))
                    result_video_writer.init("result", current_fps).start()

                if result_video_writer.is_thread_alive():
                    result_video_queue.put(image)

            if args.visualize:
                cv2.imshow("Detections", image)
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    stop_application()

            if args.debug and t.time() - dt > 5:
                print_fps(fps, True)
                dt = t.time()

        except KeyboardInterrupt:
            print("\n[CTRL+C detected] stopping application")
            stop_application()
