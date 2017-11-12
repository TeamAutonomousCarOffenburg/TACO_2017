import argparse
import os
import pprint as pp
from collections import OrderedDict

from utils.tools import read_json_file


def get_args():
    parser = argparse.ArgumentParser(
        description='Process images given by ADTF Ext Python Server Module.')

    parser.add_argument('-c', '--config', required=True)

    args = parser.parse_args()

    if not os.path.isfile(args.config):
        print("[ERROR] config file does not exist... {}".format(args.config))
        exit(1)

    return args


if __name__ == '__main__':
    args = get_args()

    config = read_json_file(args.config)

    images = 0
    tags = OrderedDict()
    for file in config:
        images += 1

        for tag in file["annotations"]:
            class_name = tag["class"]
            if tag["class"] in tags:
                tags[class_name] += 1
            else:
                tags[class_name] = 1

    ranking = sorted(tags.items(), key=lambda x: x[1], reverse=True)
    print("images: {}".format(images))
    print("###################")
    for rank in ranking:
        print("{}: {}".format(rank[0], rank[1]))
