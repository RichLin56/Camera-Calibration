#!/usr/bin/env python
import argparse
import glob
import os
import time
import sys

import cv2
import matplotlib.pyplot as plt
import numpy as np

import calib_checker
import configuration
import log.logging_config 
from misc import camera_intrinsic_from_xml, camera_instrinsics_to_xml, stereo_camera_extrinsics_to_xml

FILE_DIR = os.path.dirname(os.path.abspath(__file__))
log.logging_config.setup_logging(path_to_config=os.path.join(FILE_DIR,
                                                             'log',
                                                             'logging_config.json'))
__logger = log.logging_config.get_logger(__name__)


def extract_corners_checker_stereo(image_paths_left, image_paths_right, nr_image_pairs, checker_board, flags: dict, use_sector_based: bool, corner_sub_pixel: dict, output_dir):
    image_points_left, image_points_right, object_points_total = [], [], []
    image_size = 0
    for i in range(nr_image_pairs):
        image_pair = [image_paths_left[i], image_paths_right[i]]
        image_points, object_points, image_size, _ = calib_checker.extract_corners_checker(image_pair, checker_board, 
                                                                                        flags=flags,
                                                                                        use_sector_based=use_sector_based,
                                                                                        corner_sub_pixel=corner_sub_pixel,
                                                                                        output_dir=output_dir)
        if len(image_points) == 2:
            image_points_left.append(image_points[0])
            image_points_right.append(image_points[1])
            object_points_total.append(object_points[0])  
            image_size = image_size
    return image_points_left, image_points_right, object_points_total, image_size


def determine_checker_stereo_calibration_flag(flag_dict: dict):
    flag = 0
    if flag_dict['cv2.CALIB_FIX_INTRINSIC'] is True:
        flag += cv2.CALIB_FIX_INTRINSIC
    if flag_dict['cv2.CALIB_USE_INTRINSIC_GUESS'] is True:
        flag += cv2.CALIB_USE_INTRINSIC_GUESS
    if flag_dict['cv2.CALIB_USE_EXTRINSIC_GUESS'] is True:
        flag += cv2.CALIB_USE_EXTRINSIC_GUESS
    if flag_dict['cv2.CALIB_FIX_PRINCIPAL_POINT'] is True:
        flag += cv2.CALIB_FIX_PRINCIPAL_POINT
    if flag_dict['cv2.CALIB_FIX_FOCAL_LENGTH'] is True:
        flag += cv2.CALIB_FIX_FOCAL_LENGTH
    if flag_dict['cv2.CALIB_FIX_ASPECT_RATIO'] is True:
        flag += cv2.CALIB_FIX_ASPECT_RATIO
    if flag_dict['cv2.CALIB_SAME_FOCAL_LENGTH'] is True:
        flag += cv2.CALIB_SAME_FOCAL_LENGTH
    if flag_dict['cv2.CALIB_ZERO_TANGENT_DIST'] is True:
        flag += cv2.CALIB_ZERO_TANGENT_DIST
    if flag_dict['cv2.CALIB_FIX_K1'] is True:
        flag += cv2.CALIB_FIX_K1
    if flag_dict['cv2.CALIB_FIX_K2'] is True:
        flag += cv2.CALIB_FIX_K2
    if flag_dict['cv2.CALIB_FIX_K3'] is True:
        flag += cv2.CALIB_FIX_K3
    return flag


def calibrate_checker_stereo(object_points, image_points_left, image_points_right, camera_matrix_left, dist_coeffs_left, camera_matrix_right, dist_coeffs_right, image_size, flags: int):
    R,T = None, None

    criteria_calibration = (
                            cv2.TERM_CRITERIA_COUNT & cv2.TERM_CRITERIA_EPS, 
                            10000, 1e-5
                            )

    result = cv2.stereoCalibrateExtended(object_points, 
                                         image_points_left, 
                                         image_points_right, 
                                         camera_matrix_left, 
                                         dist_coeffs_left, 
                                         camera_matrix_right, 
                                         dist_coeffs_right, 
                                         image_size,
                                         R, T,
                                         flags=flags,
                                         criteria=criteria_calibration)

    __logger.info('Stereo Calibration Results:')
    __logger.info('---------------------------------')
    __logger.info('Reprojection Error:\n {}\n'.format(np.array(result[0])))
    __logger.info('Camera Matrix Left:\n{}\n'.format(np.array(result[1])))
    __logger.info('Distortion Coefficients Left:\n{}\n'.format(np.transpose(result[2])))
    __logger.info('Camera Matrix Right:\n{}'.format(np.array(result[3])))
    __logger.info('Distortion Coefficients Right:\n{}\n'.format(np.transpose(result[4])))
    __logger.info('Rotation Matrix from Left to Right:\n{}\n'.format(np.transpose(result[5])))
    __logger.info('Translation Vector from Left to Right:: \n{}\n'.format(np.transpose(result[6])))
    __logger.info('Essential Matrix:\n{}\n'.format(np.transpose(result[7])))
    __logger.info('Fundamental Matrix :\n{}'.format(np.transpose(result[8])))
    __logger.info('Per View Reprojection Error:\n{}\n'.format(np.transpose(result[9])))

    return result


def main():
    parser = argparse.ArgumentParser(formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    parser.add_argument('--config', help='path to config.json', default='config_checker_stereo.jsonc')    
    args = parser.parse_args()
    # Load config
    if not os.path.exists(args.config):
        __logger.info('Config path {} does not exist:'.format(args.config))
        sys.exit(1)    

    cfg = configuration.load_config_json(args.config)
    input_dir = cfg['io_settings']['input_dir']
    output_dir = cfg['io_settings']['output_dir']
    img_extension = cfg['io_settings']['image_extension']  

    path_to_intrics_left = cfg['io_settings']['intrinsics_camera_left']
    path_to_intrics_right = cfg['io_settings']['intrinsics_camera_right']
    # Read camera intrinsics
    camera_matrix_left, dist_coeffs_left, _ =  camera_intrinsic_from_xml(path_to_intrics_left)

    __logger.info('\nCamera Matrix Left:\n{}\n'.format(np.array(camera_matrix_left)))
    __logger.info('\nDistortion Coefficients Left:\n{}\n'.format(dist_coeffs_left))

    camera_matrix_right, dist_coeffs_right, _ =  camera_intrinsic_from_xml(path_to_intrics_right)
    
    __logger.info('\nCamera Matrix Right:\n{}\n'.format(np.array(camera_matrix_right)))
    __logger.info('\nDistortion Coefficients Right:\n{}\n'.format(dist_coeffs_right))

    # Create board
    checker_board = calib_checker.create_board_checker(**cfg['checker_board'])

    # Get image paths
    image_paths = sorted(glob.glob(os.path.join(input_dir, '*{}'.format("png"))))
    image_paths_left = [image_path for image_path in image_paths if "left" in image_path] 
    image_paths_right = [image_path for image_path in image_paths if "right" in image_path] 
    assert len(image_paths_left) == len(image_paths_right), "len(image_paths_left) == len(image_paths_right)" 

    # Extract corners
    image_points_left, image_points_right, object_points, image_size = extract_corners_checker_stereo(image_paths_left, 
                                                                                                      image_paths_right, 
                                                                                                      int(len(image_paths)/2), 
                                                                                                      checker_board, 
                                                                                                      **cfg['checker_corner_extraction_settings'], 
                                                                                                      output_dir=output_dir)
                                                                                      
    # Calibrate stereo camera extrinsics 
    result = calibrate_checker_stereo(object_points, 
                                      image_points_left, 
                                      image_points_right, 
                                      camera_matrix_left, 
                                      dist_coeffs_left, 
                                      camera_matrix_right, 
                                      dist_coeffs_right, 
                                      image_size,
                                      flags=determine_checker_stereo_calibration_flag(cfg['checker_stereo_calibration_settings']['flags']))

    # TODO: Implement refinement based on max_rep_error (when time is available)
    # Save result to xml
    stereo_camera_extrinsics_to_xml(result[5], result[6].reshape(-1), output_dir=output_dir)

    log.logging_config.move_log_file(__logger.handlers[0].baseFilename, os.path.join(output_dir, 'info.log'))


if __name__ == "__main__":
    np.set_printoptions(precision=6)
    main()