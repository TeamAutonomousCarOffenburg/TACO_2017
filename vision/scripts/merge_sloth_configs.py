from __future__ import absolute_import
from __future__ import division
from __future__ import print_function
import argparse
import os.path
import glob

from utils import tools
from object_detection.utils import label_map_util


def main():
    parser = argparse.ArgumentParser(description='Process some integers.')
    parser.add_argument('--root_image_folder', required=True)
    parser.add_argument(
        '-c',
        '--config',
        required=False,
        default="./config/vision_server.config.json",
        help=
        "Path to the configuration file with the settings for Object Detection"
    )
    args = parser.parse_args()

    if not os.path.isdir(args.root_image_folder):
        print("root_image_folder does not exist...")
        exit()

    config = tools.load_config(args.config)

    label_map = label_map_util.load_labelmap(config["label_map"])
    map_categories = label_map_util.convert_label_map_to_categories(
        label_map,
        max_num_classes=config["num_classes"],
        use_display_name=True)

    categories = []
    for category in map_categories:
        categories.append(category["name"])

    final_config = {}
    annotations_total = 0
    for file in glob.glob(os.path.join(args.root_image_folder, "*/*.json")):
        config = tools.read_json_file(file)
        annotations_count = 0

        for image in config:
            if image["filename"] not in final_config and len(
                    image["annotations"]) > 0:
                annotations = []
                for annotation in image["annotations"]:
                    if annotation["class"] in categories:
                        annotations.append(annotation)

                if len(annotations) > 0:
                    image["annotations"] = annotations
                    final_config[image["filename"]] = image
                    annotations_count += len(annotations)

        print("{} length: {} annotations: {}".format(
            file, len(config), annotations_count))
        annotations_total += annotations_count

    final_json_content = []
    for image in final_config.values():
        final_json_content.append(image)

    output_file = os.path.join(args.root_image_folder, "final_config.json")
    print("final config file: {} images: {} annotations: {}".format(
        output_file, len(final_json_content), annotations_total))
    tools.write_json_file(output_file, final_json_content)


if __name__ == '__main__':
    main()
