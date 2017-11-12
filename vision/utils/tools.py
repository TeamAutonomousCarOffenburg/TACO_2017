import json
from datetime import datetime as dt
import os


def read_json_file(file):
    f = open(file, "r")
    content = json.load(f)
    f.close()
    return content


def write_json_file(file, content):
    f = open(file, "w")
    json.dump(content, f, indent=4)
    f.close()


def get_timestamp_ms():
    # microseconds to milliseconds
    return int(dt.now().timestamp() * 1000)


def load_config(config_file):
    config = read_json_file(config_file)
    if not os.path.isfile(config["path_to_ckpt"]):
        print("the path to the ckpt file does not exist: {}".format(
            config["path_to_ckpt"]))
        exit(2)
    if not os.path.isfile(config["label_map"]):
        print("the path to the label map does not exist: {}".format(
            config["label_map"]))
        exit(3)

    return config
