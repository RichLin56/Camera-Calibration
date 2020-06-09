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

import calib_charuco
import configuration
import log.logging_config 
from misc import camera_intrinsic_from_xml, camera_instrinsics_to_xml, stereo_camera_extrinsics_to_xml

FILE_DIR = os.path.dirname(os.path.abspath(__file__))
log.logging_config.setup_logging(path_to_config=os.path.join(FILE_DIR,
                                                             'log',
                                                             'logging_config.json'))
__logger = log.logging_config.get_logger(__name__)


def extract_corners_charuco_stereo(image_paths_left: list, image_paths_right: list, nr_image_pairs: int, charuco_board, corner_sub_pixel: dict, min_corners_aruco: int, min_corners_charuco:int, output_dir):
    corners_left, corners_right, ids_left, ids_right = [], [], [], []
    for i in range(nr_image_pairs):
        image_pair = [image_paths_left[i], image_paths_right[i]]
        corners, ids, image_size, _ = calib_charuco.extract_corners_charuco(image_pair, charuco_board, corner_sub_pixel, min_corners_aruco, min_corners_charuco, output_dir) 
        if len(corners) == 2:
            corners_left.append(corners[0])
            corners_right.append(corners[1])
            ids_left.append(ids[0])
            ids_right.append(ids[1])   
    return corners_left, corners_right, ids_left, ids_right


def estimate_extrinsics_charuco(corners, ids, charuco_board, camera_matrix, dist_coeffs):
    rvecs, tvecs = [], []
    for count in range(len(corners)):
        curr_corners = corners[count]
        curr_ids = ids[count]

        rvec, tvec = 0, 0
        retval, rvec, tvec = aruco.estimatePoseCharucoBoard(curr_corners, curr_ids, charuco_board, camera_matrix, dist_coeffs, rvec, tvec)
        if retval:
            rvecs.append(rvec)
            tvecs.append(tvec)
    return rvecs, tvecs


def compose_rts(rvecs_left, tvecs_left, rvecs_right, tvecs_right):
    res_rvecs, res_tvecs = [], [] 
    results = []
    for count in range(len(rvecs_left)):
        # Calculation of R,T from left camera CS to right camera CS
        # R = R_Right * R_Left.Inverse();
        # T = T_R - R * T_L;

        r_left_mat = cv2.Rodrigues(rvecs_left[count])[0]
        r_right_mat = cv2.Rodrigues(rvecs_right[count])[0]

        t_left = tvecs_left[count]
        t_right = tvecs_right[count]

        r = np.matmul(r_right_mat, cv2.invert(r_left_mat)[-1])
        t = t_right - np.matmul(r, t_left) 
        
        res_rvecs.append(r)
        res_tvecs.append(t)    
    
    return res_rvecs, res_tvecs


def compute_final_rt(rvecs: list, tvecs: list, method: str):
    if method=='median':
        tvecs = np.array(tvecs)
        tvec = np.array([np.median(tvecs[:, i]) for i in range(3)])    
        rvecs = np.array(rvecs)
        rvec = np.array([np.median(rvecs[:, i][0:, j]) for i in range(3) for j in range(3)]).reshape(3, 3)
    return rvec, tvec


def main():
    parser = argparse.ArgumentParser(formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    parser.add_argument('--config', help='path to config.json', default='config_charuco_stereo.jsonc')    
    args = parser.parse_args()

    if not os.path.exists(args.config):
        __logger.info('Config path {} does not exist:'.format(args.config))
        sys.exit(1)    

    cfg = configuration.load_config_json(args.config)
    input_dir = cfg['io_settings']['input_dir']
    output_dir = cfg['io_settings']['output_dir']
    img_extension = cfg['io_settings']['image_extension']  

    path_to_intrics_left = cfg['io_settings']['intrinsics_camera_left']
    path_to_intrics_right = cfg['io_settings']['intrinsics_camera_right']

    camera_matrix_left, dist_coeffs_left, _ =  camera_intrinsic_from_xml(path_to_intrics_left)

    __logger.info('\nCamera Matrix Left:\n{}'.format(np.array(camera_matrix_left)))
    __logger.info('\nDistortion Coefficients Left:\n{}'.format(dist_coeffs_left))

    camera_matrix_right, dist_coeffs_right, _ =  camera_intrinsic_from_xml(path_to_intrics_right)
    
    __logger.info('\nCamera Matrix Right:\n{}'.format(np.array(camera_matrix_right)))
    __logger.info('\nDistortion Coefficients Right:\n{}'.format(dist_coeffs_right))

    # Create board
    charuco_board = calib_charuco.create_board_charuco(**cfg['charuco_board'])

    # Get image paths
    image_paths = sorted(glob.glob(os.path.join(input_dir, '*{}'.format("png"))))
    image_paths_left = [image_path for image_path in image_paths if "left" in image_path] 
    image_paths_right = [image_path for image_path in image_paths if "right" in image_path] 
    assert len(image_paths_left) == len(image_paths_right), "len(image_paths_left) == len(image_paths_right)" 

    # Extract corners
    corners_left, corners_right, ids_left, ids_right = extract_corners_charuco_stereo(image_paths_left, 
                                                                                      image_paths_right, 
                                                                                      int(len(image_paths)/2), 
                                                                                      charuco_board, 
                                                                                      **cfg['charuco_corner_extraction_settings'],
                                                                                      output_dir=output_dir)   
                                                                                      
    # Calibrate stereo camera extrinsics 
    rvecs_left, tvecs_left = estimate_extrinsics_charuco(corners_left, ids_left, charuco_board, camera_matrix_left, dist_coeffs_left)
    __logger.info('\nEstimated Rotation Matrix Left:\n{}'.format(np.array(rvecs_left)))
    __logger.info('\nEstimated Translation Vectors Left:\n{}'.format(np.array(rvecs_left)))
    rvecs_right, tvecs_right = estimate_extrinsics_charuco(corners_right, ids_right, charuco_board, camera_matrix_right, dist_coeffs_right)
    __logger.info('\nEstimated Rotation Matrix Right:\n{}'.format(np.array(rvecs_right)))
    __logger.info('\nEstimated Translation Vectors Right:\n{}'.format(np.array(rvecs_right)))
    res_rvecs, res_tvecs = compose_rts(rvecs_left, tvecs_left, rvecs_right, tvecs_right)
    __logger.info('\Calculated Rotations From Left to Right:\n{}'.format(np.array(res_rvecs)))
    __logger.info('\Calculated Translations From Left to Right:\n{}'.format(np.array(res_tvecs)))
    # TODO: Median method is good, but i think it is better to use median as starting point for some optimization method
    final_rvec, final_tvec = compute_final_rt(res_rvecs, res_tvecs, method='median')
    __logger.info('\nFinal Calculated({0}) Rotation From Left to Right:\n{1}'.format('median', np.array(final_rvec)))
    __logger.info('\nFinal Calculated({0}) Rotation(Rodrigues) From Left to Right:\n{1}'.format('median', np.array(cv2.Rodrigues(final_rvec)[0])))
    __logger.info('\nFinal Calculated({0}) Translation From Left to Right:\n{1}'.format('median', np.array(final_tvec)))

    # Save result to xml
    stereo_camera_extrinsics_to_xml(final_rvec, final_tvec, output_dir=output_dir)

    log.logging_config.move_log_file(__logger.handlers[0].baseFilename, os.path.join(output_dir, 'info.log'))


if __name__ == "__main__":
    np.set_printoptions(precision=6)
    main()
                   
