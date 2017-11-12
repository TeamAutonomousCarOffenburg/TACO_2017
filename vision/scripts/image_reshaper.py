import argparse
import os.path
from PIL import Image


def main():
    parser = argparse.ArgumentParser(description='Process some images.')
    parser.add_argument('--source-dir', required=True)
    parser.add_argument('--target-dir', required=True)
    args = parser.parse_args()

    if not os.path.isdir(args.source_dir):
        print("Source folder does not exist")
        exit()
    if not os.path.isdir(args.target_dir):
        print("Target folder does not exist")
        exit()

    for image in os.listdir(args.source_dir):
        img = Image.open(os.path.join(args.source_dir, image))
        img_shaped = img.crop((30, 300, 1270, 630))
        img_shaped.save(os.path.join(args.target_dir, image))


if __name__ == '__main__':
    main()
