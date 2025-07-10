from PIL import Image
import os
import argparse
import pathlib
from distutils.dir_util import copy_tree
import datetime


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('-i', help='Input directory', required=True, type=lambda p: pathlib.Path(p).absolute())
    parser.add_argument('-o', help='Output directory', required=True, type=lambda p: pathlib.Path(p).absolute())
    parser.add_argument('-c','--color', help='<Required> color rgb value to be removed. Specify as r g b. E.g. -c 255 225 214', required=True, nargs=3, type=int)
    args = parser.parse_args()
    print(args)
    # Define the directory containing your PNG files
    input_dir = args.i
    output_dir = args.o
    removal_color = args.color
    
    if input_dir == output_dir:
        print("WARNING! INPUT AND OUTPUT DIRECTORIES ARE THE SAME. MAKING A BACKUP OF THE INPUT DIRECTORY")
        suffix = 'backup-' + datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        copy_tree(input_dir, os.path.join(input_dir, suffix))

    # Ensure the output directory exists
    os.makedirs(output_dir, exist_ok=True)

    # Choose which color channel to zero out ('R', 'G', or 'B')
    removal_color = args.color

    # Channel index map

    # Loop through all PNG files in the directory
    for filename in os.listdir(input_dir):
        if filename.endswith(".png"):
            file_path = os.path.join(input_dir, filename)

            # Open the image
            img = Image.open(file_path).convert('RGBA')
            pixels = img.load()

            # Process each pixel
            for i in range(img.width):
                for j in range(img.height):
                    r, g, b, a = pixels[i, j]
                    if r == removal_color[0] and g == removal_color[1] and b == removal_color[2]:
                        pixels[i, j] = (0, 0, 0, 0)


            # Save the modified image to the output directory
            output_path = os.path.join(output_dir, filename)
            img.save(output_path)

    print(f"Processed images are saved in '{output_dir}'.")


if __name__ == '__main__':
    main()