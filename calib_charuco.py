#!/usr/bin/env python
import argparse
import glob
import os
import time
import sys

import cv2
import matplotlib.pyplot as plt
import numpy as np

from cv2 import aruco

import configuration
import log.logging_config 
from misc import largest_prime_factor, camera_instrinsics_to_xml

FILE_DIR = os.path.dirname(os.path.abspath(__file__))
log.logging_config.setup_logging(path_to_config=os.path.join(FILE_DIR,
                                                             'log',
                                                             'logging_config.json'))
__logger = log.logging_config.get_logger(__name__)


def create_board_charuco(aruco_dict_int: int, squares_x, squares_y, square_length_mm, marker_length_mm, print_scaling):
    # chose dictionary 
    aruco_dict = aruco.Dictionary_get(aruco_dict_int)
    # create charuco board from dictionary
    charuco_board = aruco.CharucoBoard_create(squares_x, squares_y, square_length_mm * print_scaling, marker_length_mm * print_scaling, aruco_dict)

    return charuco_board


def extract_corners_charuco(image_paths: list, charuco_board, corner_sub_pixel: dict, min_corners_aruco: int, min_corners_charuco: int,output_dir=None):
    charuco_corners = list()
    charuco_ids = list()  
    image_sizes = list()  
    image_paths_used = list()
    # From OpenCV-docu:
    #
    # Criteria for termination of the iterative process of corner refinement. 
    # That is, the process of corner position refinement stops either after criteria.maxCount iterations 
    # or when the corner position moves by less than criteria.epsilon on some iteration.
    criteria_subpixel = (cv2.TERM_CRITERIA_MAX_ITER + cv2.TERM_CRITERIA_EPS , 10000, 1e-5)
    for path in image_paths:
        __logger.info('Processing image {0}'.format(os.path.basename(path)))
        # Load image
        img = cv2.imread(path)
        img_gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        # Detect aruco markers on board
        corners_aruco, ids_aruco, rejected_corners_aruco = cv2.aruco.detectMarkers(img_gray, charuco_board.dictionary)
        
        if len(corners_aruco) > min_corners_aruco:
            # Sub pixel detection
            for corner in corners_aruco:
                cv2.cornerSubPix(img_gray, corner,
                                 winSize = tuple(corner_sub_pixel['winSize']),
                                 zeroZone = tuple(corner_sub_pixel['zeroZone']),
                                 criteria = criteria_subpixel)
            # Interpolate charuco corners
            nr_corners, corners, ids = aruco.interpolateCornersCharuco(corners_aruco, ids_aruco, img_gray, charuco_board)
            
            # Verify corners are detected
            if corners is not None and ids is not None and nr_corners >= min_corners_aruco:
                charuco_corners.append(corners)
                charuco_ids.append(ids)
                height, width = img_gray.shape[:2]
                image_sizes.append((width, height))
                image_paths_used.append(path)
                if output_dir:
                    prime_factor = largest_prime_factor(nr_corners)
                    os.makedirs(os.path.join(output_dir, 'CornerVisualization'), exist_ok=True)
                    img = aruco.drawDetectedCornersCharuco(img, corners, ids, cornerColor=(0, 200, 0))
                    cv2.imwrite(os.path.join(output_dir, 'CornerVisualization', os.path.basename(path)), img)
                    img = cv2.drawChessboardCorners(img, (prime_factor, int(nr_corners/prime_factor)), corners, True)
                    cv2.imwrite(os.path.join(output_dir, 'CornerVisualization', "_colored_" + os.path.basename(path)), img)
            else:
                __logger.info('Rejecting image, less than {} charuco corners detected (after interpolation)...'.format(min_corners_charuco)) 
        else:
            __logger.info('Rejecting image, less than {} aruco corners detected...'.format(min_corners_aruco))   

    # verify images are same size    
    if not len(set(image_sizes)) == 1:
        __logger.info('Images are not of same size! Exiting...')
        sys.exit(1)
    return charuco_corners, charuco_ids, list(set(image_sizes))[0], image_paths_used


def determine_charuco_calibration_flag(flag_dict: dict):
    flag = 0
    if flag_dict['cv2.CALIB_USE_INTRINSIC_GUESS'] is True:
        flag += cv2.CALIB_USE_INTRINSIC_GUESS
    if flag_dict['cv2.CALIB_FIX_PRINCIPAL_POINT'] is True:
        flag += cv2.CALIB_FIX_PRINCIPAL_POINT
    if flag_dict['cv2.CALIB_RATIONAL_MODEL'] is True:
        flag += cv2.CALIB_RATIONAL_MODEL
    if flag_dict['cv2.CALIB_FIX_ASPECT_RATIO'] is True:
        flag += cv2.CALIB_FIX_ASPECT_RATIO
    if flag_dict['cv2.CALIB_FIX_K1'] is True:
        flag += cv2.CALIB_FIX_K2
    if flag_dict['cv2.CALIB_FIX_K2'] is True:
        flag += cv2.CALIB_FIX_K2
    if flag_dict['cv2.CALIB_FIX_K3'] is True:
        flag += cv2.CALIB_FIX_K3
    return flag


def calibrate_charuco(charuco_board, corners, ids, image_size, flags: int, focal_length_mm: float, pixel_size_mm: float, camera_matrix_init=None, dist_coeffs_init=None):
    # In camera matrix the focal lengths fx,fy are expressed in pixel units
    # W: is the sensor width expressed in world units, let's say mm
    # Wp: is the width of a pixel on the chip expressed in world units, let's say mm
    # w: is the image width expressed in pixel
    # W = Wp * w
    # Fx: is the focal length expressed in world units (usually mm)
    # fx: is the focal length expressed in pixel units (as is in the camera matrix)
    # Fx = fx * W /w = = fx * Wp * w / w = fx * Wp
    # Fy = fy * H /h =  fy * Hp
    # fx = Fx / Wp
    # fy = Fy / Hp

    fx = focal_length_mm / pixel_size_mm
    fy = focal_length_mm / pixel_size_mm

    if camera_matrix_init is None:
        camera_matrix_init = np.array([[  fx,     0.,     image_size[0]/2.],
                                        [  0.,     fy,     image_size[1]/2.],
                                        [  0.,     0.,     1.]])   

    if dist_coeffs_init is None:
        dist_coeffs_init = np.zeros((5,1))

    criteria_calibration = (
                            cv2.TERM_CRITERIA_COUNT & cv2.TERM_CRITERIA_EPS, 
                            10000, 1e-5
                            )

    result = cv2.aruco.calibrateCameraCharucoExtended(charucoCorners=corners,
                                                      charucoIds=ids,
                                                      board=charuco_board,
                                                      imageSize=image_size,
                                                      cameraMatrix=camera_matrix_init,
                                                      distCoeffs=dist_coeffs_init,
                                                      flags=flags,
                                                      criteria=criteria_calibration)
    
    __logger.info('Calibration Results:')
    __logger.info('---------------------------------')
    __logger.info('\nReprojection Error:\n {}\n'.format(np.array(result[0])))
    __logger.info('\nCamera Matrix:\n{}\n'.format(np.array(result[1])))
    __logger.info('\nDistortion Coefficients:\n{}\n'.format(np.transpose(result[2])))
    # TODO: Implement option (in config-file) to display these values in log-file
    #__logger.info('\nRotation Vectors:\n{}\n'.format(np.transpose(result[3])))
    #__logger.info('\nTranslation Vectors:\n{}\n'.format(np.transpose(result[4])))
    #__logger.info('\nStd Deviation Intrinsics:\n{}\n'.format(np.transpose(result[5])))
    #__logger.info('\nStd Deviation Extrinsics:\n{}\n'.format(np.transpose(result[6])))
    __logger.info('\nPer View Reprojection Error:\n{}\n'.format(np.transpose(result[7])))

    return result


def calibrate_charuco_refine(charuco_board, corners, ids, image_size, flags: int, focal_length_mm: float, pixel_size_mm: float, camera_matrix_init, dist_coeffs_init, initial_rep_errors: list, max_rep_err: float):
    __logger.info('Starting charuco calibration refinement...')
    __logger.info('Rejecting all images with reprojection_error < {0} ...'.format(max_rep_err))
    corners_refine, ids_refine = [], []
    for i in range(len(corners)):
        if initial_rep_errors[i] < max_rep_err:
            corners_refine.append(corners[i])
            ids_refine.append(ids[i])
    __logger.info('Using {0}/{1} images ...'.format(len(corners_refine), len(corners)))
    result = calibrate_charuco(charuco_board, 
                               corners_refine, 
                               ids_refine, 
                               image_size, 
                               flags, 
                               focal_length_mm=focal_length_mm, 
                               pixel_size_mm=pixel_size_mm, 
                               camera_matrix_init=camera_matrix_init, 
                               dist_coeffs_init=dist_coeffs_init)  
    return result


def main():
    parser = argparse.ArgumentParser(formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    parser.add_argument('--config', help='path to config.jsonc', default='config_charuco.jsonc')    
    args = parser.parse_args()

    if not os.path.exists(args.config):
        __logger.info('Config path {} does not exist:'.format(args.config))
        sys.exit(1)    

    cfg = configuration.load_config_json(args.config)
    input_dir = cfg['io_settings']['input_dir']
    output_dir = cfg['io_settings']['output_dir']
    img_extension = cfg['io_settings']['image_extension']

    # create board
    charuco_board = create_board_charuco(**cfg['charuco_board'])

    # get image paths
    image_paths = glob.glob(os.path.join(input_dir, '*{}'.format(img_extension)))

    # extract corners
    corners, ids, image_size, _ = extract_corners_charuco(image_paths, charuco_board, **cfg['charuco_corner_extraction_settings'], output_dir=output_dir)

    # calibrate camera
    result = calibrate_charuco(charuco_board, corners, ids, image_size, determine_charuco_calibration_flag(cfg['charuco_calibration_settings']['flags']), 
                               **cfg['camera_parameter'])  

    # refine calibration results
    result = calibrate_charuco_refine(charuco_board, corners, ids, image_size, 
                                      determine_charuco_calibration_flag(cfg['charuco_calibration_settings']['flags']), 
                                      **cfg['camera_parameter'], 
                                      camera_matrix_init=result[1], 
                                      dist_coeffs_init=result[2], 
                                      initial_rep_errors=result[-1],
                                      max_rep_err=cfg['charuco_calibration_settings']['max_reprojection_error'])   

    # save result to xml
    camera_instrinsics_to_xml(result[1], result[2].reshape(-1), cfg['camera_parameter']['pixel_size_mm'], output_dir=output_dir)

    log.logging_config.move_log_file(__logger.handlers[0].baseFilename, os.path.join(output_dir, 'info.log'))


if __name__ == "__main__":
    np.set_printoptions(precision=6)
    main()


