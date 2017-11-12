from __future__ import absolute_import
from __future__ import division
from __future__ import print_function
import argparse

import os.path
import glob

from utils import tools


def main():
    parser = argparse.ArgumentParser(description='Process some integers.')
    parser.add_argument('--root_image_folder', required=True)
    args = parser.parse_args()

    if not os.path.isdir(args.root_image_folder):
        print("root_image_foldere does not exist...")
        exit()

    for file in glob.glob(os.path.join(args.root_image_folder, "*/*.json")):
        sloth_config = []
        config = tools.read_json_file(file)

        for image in config:
            if image["filename"] not in sloth_config and len(
                    image["annotations"]) > 0:
                sloth_config.append(image)
        print("file: {}".format(file))
        tools.write_json_file(file, sloth_config)


if __name__ == '__main__':
    main()
