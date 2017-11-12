import argparse
import glob
import os
import time

import cv2

from utils import tools
from utils import visualisation as v
from utils.tf_object_detection import ObjectDetection

extensions = ['jpg', 'jpeg', 'JPG', 'JPEG', 'png', 'PNG']


def get_args():
    parser = argparse.ArgumentParser(description='Process some integers.')
    parser.add_argument('--image-dir', required=True)
    parser.add_argument('--output-dir', required=True)
    parser.add_argument('--validate-json', required=True)
    parser.add_argument('-g', '--graph', required=False)
    parser.add_argument(
        '-c',
        '--config',
        required=False,
        default="./config/vision_server.config.json",
        help=
        "Path to the configuration file with the settings for Object Detection"
    )
    parser.add_argument('-iw', '--image-width', type=int, default=1240)
    parser.add_argument('-ih', '--image-height', type=int, default=330)
    parser.add_argument('-ic', '--image-channels', type=int, default=3)
    parser.add_argument(
        '-gu',
        '--gpu-usage',
        type=float,
        default=0.5,
        help="How much gpu can we use?")
    args = parser.parse_args()

    if not os.path.isdir(args.image_dir):
        print("[ERROR] Image dir does not exist... {}".format(args.image_dir))

    if os.path.isdir(args.output_dir):
        for file in os.scandir(args.output_dir):
            os.unlink(file.path)
    else:
        os.makedirs(args.output_dir)

    return args


def main():
    args = get_args()
    validate_images = []

    for extension in extensions:
        for file in glob.glob(args.image_dir + '*.' + extension):
            validate_images.append(file)

    config = tools.load_config(args.config)

    validation_config = tools.read_json_file(args.validate_json)

    config_images = {}
    for image in validation_config:
        config_images[image["filename"]] = image["annotations"]

    if len(config_images) == 0:
        print("no images found in config...")
        exit()

    if args.graph is not None:
        config["path_to_ckpt"] = args.graph

    od = ObjectDetection(config, args.image_width, args.image_height,
                         args.gpu_usage)
    od.start_up()

    total_classes_in_config = 0
    predicted_valid = 0
    predicted_false = 0
    prediction_time = 0
    validated_images = 0
    empty_result = 0

    validate_images.sort()
    for image in validate_images:
        image_file = os.path.basename(image)

        if image_file not in config_images.keys():
            #print("Skip image {} cause not in config...".format(image_file))
            continue
        else:
            validated_images += 1

        imgcv = cv2.imread(image)
        start = time.time()
        result = od.detect_objects(imgcv)
        end = time.time()

        total_classes_in_config += len(config_images[image_file])

        if len(result["recognizedObjects"]) > 0:
            objects = result["recognizedObjects"]
            result_image = os.path.join(args.output_dir,
                                        "result_" + image_file)
            prediction_time += (end - start)

            imgcv = v.draw_boxes_and_labels(imgcv, objects)
            cv2.imwrite(result_image, imgcv)

            for object in objects:
                if result_in_annotations(config_images[image_file],
                                         object["class_name"]):
                    predicted_valid += 1
                    # print("valid: {}".format(image_file))
                else:
                    predicted_false += 1
                    print("{} predicted not tagged class: {}".format(
                        image_file, object["class_name"]))
        else:
            empty_result += 1

    print(
        "validation classes: {} valid: {} false: {} empty results: {} total: {:.2f}% avg_time {:.2f}s".
        format(total_classes_in_config, predicted_valid, predicted_false,
               empty_result, predicted_valid / total_classes_in_config * 100,
               prediction_time / validated_images))

    od.shutdown()


def result_in_annotations(annotations, result_class):
    for annotation in annotations:
        if result_class == annotation["class"]:
            return True

    return False


if __name__ == '__main__':
    main()
