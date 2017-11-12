import argparse
import os
import time as t
import cv2
from utils.net_video_stream import NetVideoStream


def get_args():
    parser = argparse.ArgumentParser(
        description='Process images given by ADTF Ext Python Server Module.')

    parser.add_argument(
        '-ip',
        '--server-ip',
        default="127.0.0.1",
        help="Server IP from the Camera Relay")
    parser.add_argument(
        '-sp', '--server-port', type=int, default=1337, help="Port to connect")

    parser.add_argument('-iw', '--image-width', type=int, default=1240)
    parser.add_argument('-ih', '--image-height', type=int, default=330)
    parser.add_argument('-ic', '--image-channels', type=int, default=3)
    parser.add_argument(
        '-v',
        '--visualize',
        action='store_true',
        help="Displays the images with the results")
    parser.add_argument('-d', '--debug', action='store_true')
    parser.add_argument('-vf', '--video-folder', default="/home/aadc/videos")
    parser.add_argument(
        '-vqs',
        '--video-queue-size',
        type=int,
        default=600,
        help=
        'Size of the video queue. Contains Images which will be written to the video'
    )

    args = parser.parse_args()

    if not os.path.isdir(args.video_folder):
        print("[ERROR] folder for output vidoes does not exist... {}".format(
            args.video_folder))
        exit(1)

    return args


if __name__ == '__main__':
    args = get_args()

    stream = NetVideoStream(args.server_ip, args.server_port, args.image_width,
                            args.image_height, args.image_channels)
    if not stream.start():
        print("[ERROR] could not connect to Server...")
        exit(2)

    fourcc = cv2.VideoWriter_fourcc(*"MJPG")
    file = os.path.join(args.video_folder, 'output.avi')
    out = cv2.VideoWriter(file, fourcc, 30.0, (args.image_width,
                                               args.image_height))

    while True:
        if not stream.is_connected():
            t.sleep(0.02)
            continue

        image = stream.read()
        if image is False:
            t.sleep(0.02)
            continue

        out.write(image)
        cv2.imshow('frame', image)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    out.release()
    cv2.destroyAllWindows()
    stream.stop()
    t.sleep(1)
    stream.disconnect()
