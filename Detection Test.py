import argparse
import sys
import time

import cv2
import numpy as np
from scipy import ndimage
from skimage.morphology import skeletonize, dilation, square

import bosdyn.client
import bosdyn.client.util
from bosdyn.api import image_pb2
from bosdyn.client.image import ImageClient, build_image_request

ROTATION_ANGLE = {
    'back_fisheye_image': 0,
    'frontleft_fisheye_image': -78,
    'frontright_fisheye_image': -102,
    'left_fisheye_image': 0,
    'right_fisheye_image': 180
}


def pixel_format_type_strings():
    names = image_pb2.Image.PixelFormat.keys()
    return names[1:]


def pixel_format_string_to_enum(enum_string):
    return dict(image_pb2.Image.PixelFormat.items()).get(enum_string)


def main(argv):
    # Parse args
    parser = argparse.ArgumentParser()
    bosdyn.client.util.add_base_arguments(parser)
    parser.add_argument('--list', help='list image sources',
                        action='store_true')
    parser.add_argument('--auto-rotate', help='rotate right and front images to be upright',
                        action='store_true')
    parser.add_argument('--image-sources',
                        help='Get image from source(s)', action='append')
    parser.add_argument('--image-service', help='Name of the image service to query.',
                        default=ImageClient.default_service_name)
    parser.add_argument(
        '--pixel-format', choices=pixel_format_type_strings(),
        help='Requested pixel format of image. If supplied, will be used for all sources.')

    options = parser.parse_args()

    # Create robot object with an image client.
    sdk = bosdyn.client.create_standard_sdk('image_capture')
    robot = sdk.create_robot(options.hostname)
    bosdyn.client.util.authenticate(robot)
    robot.sync_with_directory()
    robot.time_sync.wait_for_sync()

    image_client = robot.ensure_client(options.image_service)

    # Raise exception if no actionable argument provided
    if not options.list and not options.image_sources:
        parser.error(
            'Must provide actionable argument (list or image-sources).')

    # Optionally list image sources on robot.
    if options.list:
        image_sources = image_client.list_image_sources()
        print('Image sources:')
        for source in image_sources:
            print('\t' + source.name)
    counter = 0
    # Optionally capture one or more images.
    while True:
        if options.image_sources:
            # Capture and save images to disk
            pixel_format = pixel_format_string_to_enum(options.pixel_format)
            image_request = [
                build_image_request(source, pixel_format=pixel_format)
                for source in options.image_sources
            ]
            image_responses = image_client.get_image(image_request)
    
            for image in image_responses:
                num_bytes = 1  # Assume a default of 1 byte encodings.
                if image.shot.image.pixel_format == image_pb2.Image.PIXEL_FORMAT_DEPTH_U16:
                    dtype = np.uint16
                    extension = '.png'
                else:
                    if image.shot.image.pixel_format == image_pb2.Image.PIXEL_FORMAT_RGB_U8:
                        num_bytes = 3
                    elif image.shot.image.pixel_format == image_pb2.Image.PIXEL_FORMAT_RGBA_U8:
                        num_bytes = 4
                    elif image.shot.image.pixel_format == image_pb2.Image.PIXEL_FORMAT_GREYSCALE_U8:
                        num_bytes = 1
                    elif image.shot.image.pixel_format == image_pb2.Image.PIXEL_FORMAT_GREYSCALE_U16:
                        num_bytes = 2
                    dtype = np.uint8
                    extension = '.jpg'
    
                img = np.frombuffer(image.shot.image.data, dtype=dtype)
                if image.shot.image.format == image_pb2.Image.FORMAT_RAW:
                    try:
                        # Attempt to reshape array into an RGB rows X cols shape.
                        img = img.reshape(
                            (image.shot.image.rows, image.shot.image.cols, num_bytes))
                    except ValueError:
                        # Unable to reshape the image data, trying a regular decode.
                        img = cv2.imdecode(img, -1)
                else:
                    img = cv2.imdecode(img, -1)
    
                if options.auto_rotate:
                    img = ndimage.rotate(img, ROTATION_ANGLE[image.source.name])


                hsv_image = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

                lower_red1 = np.array([0, 120, 70])
                upper_red1 = np.array([10, 255, 255])
                mask_red1 = cv2.inRange(hsv_image, lower_red1, upper_red1)

                lower_red2 = np.array([170, 120, 70])
                upper_red2 = np.array([180, 255, 255])
                mask_red2 = cv2.inRange(hsv_image, lower_red2, upper_red2)

                mask_red = mask_red1 + mask_red2

                result_image = cv2.bitwise_and(img, img, mask=mask_red)

                gray_img = cv2.cvtColor(result_image, cv2.COLOR_RGB2GRAY)
                
                _, black_white = cv2.threshold(gray_img, 1, 255, cv2.THRESH_BINARY)

                
                cv2.imshow('Original', img)
                cv2.imshow('Final', black_white)
                cv2.waitKey(15000)
                cv2.destroyAllWindows()

                time.sleep(4.0)
            
                # Save the image from the GetImage request to the current directory with the filename
                # matching that of the image source.
                image_saved_path = image.source.name
                # image_saved_path = image_saved_path.replace(
                #     '/', '')  # Remove any slashes from the filename the image is saved at locally.
                cv2.imwrite('C:\\Users\\chapmanm\\Desktop\\Ribbon_images\\'+ image_saved_path + str(counter) + extension, img)
                print("Wrote image: ", image_saved_path)
                counter += 1
                time.sleep(1)
                
    return True


if __name__ == '__main__':
    # main()
    # sys.argv = ['--image-source','frontright_fisheye_image','192.168.80.3']
    sys.argv = ['get_image.py', '--image-sources', 'hand_color_image',
                '--pixel-format', 'PIXEL_FORMAT_RGB_U8', '192.168.80.3']

    print(sys.argv)
    if not main(sys.argv[1:]):
        sys.exit(1)


