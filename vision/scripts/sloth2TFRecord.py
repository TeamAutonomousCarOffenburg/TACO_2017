from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

import argparse
import hashlib
import os
import os.path
import time

import scipy.ndimage
import tensorflow as tf

from utils import tools
from object_detection.utils import dataset_util
from object_detection.utils import label_map_util

extensions = ['jpg', 'jpeg', 'JPG', 'JPEG', 'png', 'PNG']


def main():
    parser = argparse.ArgumentParser(description='Process some integers.')
    parser.add_argument('--sloth-cfg', required=True)
    parser.add_argument('--record-file', required=True)
    parser.add_argument('--image-dir', required=True)
    parser.add_argument('--labels', required=True)
    args = parser.parse_args()

    if not os.path.isfile(args.sloth_cfg):
        print("Config-File does not exist...")
        exit()
    if not os.path.isdir(args.image_dir):
        print("Image dir does not exist...")
        exit()
    if not os.path.isfile(args.labels):
        print("Labels-File does not exist...")
        exit()

    writer = tf.python_io.TFRecordWriter(args.record_file)

    label_map_dict = label_map_util.get_label_map_dict(args.labels)
    config = tools.read_json_file(args.sloth_cfg)

    images_config = []
    images_count = 0
    annotation_count = 0
    start = time.time()
    for image in config:
        image_file = os.path.join(args.image_dir, image["filename"])
        if not os.path.isfile(image_file):
            print("image {} not found on directory".format(image["filename"]))
            continue
        extension = image_file.split(".")[-1]
        if extension not in extensions:
            print("image {} has wrong extension...".format(extension))
            continue
        if len(image["annotations"]) == 0:
            print("image {} has no annotations".format(image["filename"]))
            continue

        images_config.append(image["filename"])
        images_count += 1

        image_height, image_width, channels = scipy.ndimage.imread(
            image_file).shape

        encoded_jpg = tf.gfile.GFile(image_file, 'rb').read()
        key = hashlib.sha256(encoded_jpg).hexdigest()

        xmin = []
        ymin = []
        xmax = []
        ymax = []
        classes = []
        classes_text = []

        for annotation in image["annotations"]:
            annotation_count += 1
            a_xmin, a_xmax, a_ymin, a_ymax = fix_outside_roi_points(
                annotation, image_width, image_height)

            xmin.append(float(a_xmin) / image_width)
            ymin.append(float(a_ymin) / image_height)
            xmax.append(float(a_xmax) / image_width)
            ymax.append(float(a_ymax) / image_height)

            classes_text.append(annotation["class"].encode('utf8'))
            classes.append(label_map_dict[annotation["class"]])

        example = tf.train.Example(features=tf.train.Features(
            feature={
                'image/height':
                dataset_util.int64_feature(image_height),
                'image/width':
                dataset_util.int64_feature(image_width),
                'image/filename':
                dataset_util.bytes_feature(image["filename"].encode('utf8')),
                'image/source_id':
                dataset_util.bytes_feature(image['filename'].encode('utf8')),
                'image/key/sha256':
                dataset_util.bytes_feature(key.encode('utf8')),
                'image/encoded':
                dataset_util.bytes_feature(encoded_jpg),
                'image/format':
                dataset_util.bytes_feature(
                    image["filename"].split(".")[-1].encode('utf8')),
                'image/object/bbox/xmin':
                dataset_util.float_list_feature(xmin),
                'image/object/bbox/xmax':
                dataset_util.float_list_feature(xmax),
                'image/object/bbox/ymin':
                dataset_util.float_list_feature(ymin),
                'image/object/bbox/ymax':
                dataset_util.float_list_feature(ymax),
                'image/object/class/text':
                dataset_util.bytes_list_feature(classes_text),
                'image/object/class/label':
                dataset_util.int64_list_feature(classes),
            }))

        writer.write(example.SerializeToString())

    writer.close()

    end = time.time()
    print("conversion done in {} seconds for {} images and {} annotations".
          format(end - start, images_count, annotation_count))


def fix_outside_roi_points(annotation, image_width, image_height):
    xmin = annotation["x"]
    xmax = annotation["x"] + annotation["width"]
    ymin = annotation["y"]
    ymax = annotation["y"] + annotation["height"]

    if xmin < 0.0:
        xmin = 0.0

    if ymin < 0.0:
        ymin = 0.0

    if xmax > image_width:
        xmax = image_width

    if ymax > image_height:
        ymax = image_height

    return xmin, xmax, ymin, ymax


if __name__ == '__main__':
    main()
