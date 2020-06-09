#!/usr/bin/env python
import argparse
import glob
import os
import time
import sys

import cv2
import matplotlib.pyplot as plt
import numpy as np

import configuration
import log.logging_config 
from misc import largest_prime_factor, camera_instrinsics_to_xml

FILE_DIR = os.path.dirname(os.path.abspath(__file__))
log.logging_config.setup_logging(path_to_config=os.path.join(FILE_DIR,
                                                             'log',
                                                             'logging_config.json'))
__logger = log.logging_config.get_logger(__name__)


def create_board_checker(rows, cols, square_length_mm):
    return ((cols, rows), square_length_mm)


def determine_checker_corner_extraction_flag(flag_dict: dict, use_sector_based:bool):
    flag = 0
    if use_sector_based:
        if flag_dict['sector_based_method']['cv2.CALIB_CB_ACCURACY'] is True:
            flag += cv2.CALIB_CB_ACCURACY
        if flag_dict['sector_based_method']['cv2.CALIB_CB_NORMALIZE_IMAGE'] is True:
            flag += cv2.CALIB_CB_NORMALIZE_IMAGE
        if flag_dict['sector_based_method']['cv2.CALIB_CB_EXHAUSTIVE'] is True:
            flag += cv2.CALIB_CB_EXHAUSTIVE
        return flag
    else:
        if flag_dict['standard_method']['cv2.CALIB_CB_ADAPTIVE_THRESH'] is True:
            flag += cv2.CALIB_CB_ADAPTIVE_THRESH
        if flag_dict['standard_method']['cv2.CALIB_CB_NORMALIZE_IMAGE'] is True:
            flag += cv2.CALIB_CB_NORMALIZE_IMAGE
        if flag_dict['standard_method']['cv2.CALIB_CB_FILTER_QUADS'] is True:
            flag += cv2.CALIB_CB_FILTER_QUADS
        if flag_dict['standard_method']['cv2.CALIB_CB_FAST_CHECK'] is True:
            flag += cv2.CALIB_CB_FAST_CHECK
        return flag


def extract_corners_checker(image_paths: list, checker_board, flags: dict, use_sector_based: bool, corner_sub_pixel: dict, output_dir=None):   
    # Prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
    object_points = np.zeros((checker_board[0][0] * checker_board[0][1], 3), np.float32) # 3d point in real world space
    object_points[:, :2] = np.mgrid[0:checker_board[0][0], 0:checker_board[0][1]].T.reshape(-1,2)
    # Multiply by square sizes of chessboard
    object_points = object_points * checker_board[1]
    object_points = [object_points]

    image_points = list() # 2d points in image plane.
    image_sizes = list()  
    image_paths_used = list()
    # From OpenCV-docu:
    #
    # Criteria for termination of the iterative process of corner refinement. 
    # That is, the process of corner position refinement stops either after criteria.maxCount iterations 
    # or when the corner position moves by less than criteria.epsilon on some iteration.
    criteria_subpixel = (cv2.TERM_CRITERIA_MAX_ITER + cv2.TERM_CRITERIA_EPS , 10000, 1e-5)

    flags = determine_checker_corner_extraction_flag(flags, use_sector_based)

    for path in image_paths:
        __logger.info('Processing image {0}'.format(os.path.basename(path)))
        # Load image
        img = cv2.imread(path)
        img_gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)  

        # From OpenCV-docu:
        #
        # The function is analog to findchessboardCorners but uses a localized radon transformation approximated 
        # by box filters being more robust to all sort of noise, faster on larger images and is able to directly
        # return the sub-pixel position of the internal chessboard corners. 
        # The Method is based on the paper [55] "Accurate Detection and Localization of Checkerboard Corners for Calibration" 
        # demonstrating that the returned sub-pixel positions are more accurate than the one returned by cornerSubPix 
        # allowing a precise camera calibration for demanding applications.
        #
        # Alexander Duda and Udo Frese. Accurate detection and localization of checkerboard corners for calibration. 
        # In 29th British Machine Vision Conference. British Machine Vision Conference (BMVC-29), September 3-6, Newcastle, United Kingdom. BMVA Press, 2018.
        if use_sector_based:
            pattern_found, corners = cv2.findChessboardCornersSB(img_gray, checker_board[0], flags)
        else:
            pattern_found, corners = cv2.findChessboardCorners(img_gray, checker_board[0], flags)
            if pattern_found:
                corners = cv2.cornerSubPix(img_gray, corners, 
                                           winSize=tuple(corner_sub_pixel['winSize']), 
                                           zeroZone=tuple(corner_sub_pixel['zeroZone']), 
                                           criteria=criteria_subpixel)
        
        if pattern_found is True:
            image_points.append(corners)
            height, width = img_gray.shape[:2]
            image_sizes.append((width, height))
            image_paths_used.append(path)
            if output_dir:
                cv2.drawChessboardCorners(img, checker_board[0], corners, pattern_found)
                os.makedirs(os.path.join(output_dir, 'CornerVisualization'), exist_ok=True)
                cv2.imwrite(os.path.join(output_dir, 'CornerVisualization', os.path.basename(path)), img)
        else:
            __logger.info('Rejecting image, not all corners detected...')

    object_points *= len(image_points)
    # Verify images are same size    
    if not len(set(image_sizes)) == 1:
        __logger.info('Images are not of same size! Exiting...')        
        sys.exit(1)

    return image_points, object_points, list(set(image_sizes))[0], image_paths_used
                

def determine_checker_calibration_flag(flag_dict: dict):
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
        flag += cv2.CALIB_FIX_K1
    if flag_dict['cv2.CALIB_FIX_K2'] is True:
        flag += cv2.CALIB_FIX_K2
    if flag_dict['cv2.CALIB_FIX_K3'] is True:
        flag += cv2.CALIB_FIX_K3
    return flag


def calibrate_checker(checker_board, image_points, object_points, image_size, focal_length_mm: float, pixel_size_mm: float, flags: int, use_releasing_object: bool, camera_matrix_init=None, dist_coeffs_init=None):
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

    if use_releasing_object:
        # From OpenCV-docu:
        #
        # The index of the 3D object point in objectPoints[0] to be fixed. 
        # It also acts as a switch for calibration method selection. 
        # If object-releasing method to be used, pass in the parameter in the range of [1, objectPoints[0].size()-2], 
        # otherwise a value out of this range will make standard calibration method selected. 
        # Usually the top-right corner point of the calibration board grid is recommended to be fixed 
        # when object-releasing method being utilized. According to [219], two other points are also fixed. 
        # In this implementation, objectPoints[0].front and objectPoints[0].back.z are used.
        #  With object-releasing method, accurate rvecs, tvecs and newObjPoints are 
        # only possible if coordinates of these three fixed points are accurate enough.
        i_fixed_point = checker_board[0][0] - 1
    else: 
        # This is equal to standard calibrateCamera()
        i_fixed_point = 0

    # From OpenCV-docu:
    #
    # This function is an extension of calibrateCamera() with the method of releasing object 
    # which was proposed in [219]. In many common cases with inaccurate, unmeasured, roughly planar targets (calibration plates), 
    # this method can dramatically improve the precision of the estimated camera parameters. 
    # Both the object-releasing method and standard method are supported by this function.    
    result = cv2.calibrateCameraROExtended(
                                           objectPoints=object_points, 
                                           imagePoints=image_points, 
                                           imageSize=image_size, 
                                           iFixedPoint=i_fixed_point, 
                                           cameraMatrix=camera_matrix_init,
                                           distCoeffs=dist_coeffs_init, 
                                           flags=flags,
                                           criteria=criteria_calibration
                                           )
    
    __logger.info('Calibration Results:')
    __logger.info('---------------------------------')
    __logger.info('Reprojection Error:\n{}\n'.format(np.array(result[0])))
    __logger.info('Camera Matrix:\n{}\n'.format(np.array(result[1])))
    __logger.info('Distortion Coefficients:\n{}\n'.format(np.transpose(result[2])))
    # TODO: Implement option (in config-file) to display these values in log-file
    #__logger.info('Rotation Vectors:\n{}\n'.format(np.transpose(result[3])))
    #__logger.info('Translation Vectors:\n{}'.format(np.transpose(result[4])))
    #__logger.info('Std Deviation Intrinsics:\n{}\n'.format(np.transpose(result[6])))
    #__logger.info('Std Deviation Extrinsics:\n{}\n'.format(np.transpose(result[7])))
    __logger.info('Per View Reprojection Error:\n{}\n'.format(np.transpose(result[-1])))

    return result


def calibrate_checker_refine(checker_board, image_points, object_points, image_size, focal_length_mm: float, pixel_size_mm: float, flags: int, use_releasing_object: bool, camera_matrix_init, dist_coeffs_init, initial_rep_errors: list, max_rep_err: float):      
    __logger.info('Starting checker calibration refinement...')
    __logger.info('Rejecting all images with reprojection_error < {0} ...'.format(max_rep_err))
    image_points_refine, object_points_refine = [], []
    for i in range(len(image_points)):
        if initial_rep_errors[i] < max_rep_err:
            image_points_refine.append(image_points[i])
            object_points_refine.append(object_points[i])
    __logger.info('Using {0}/{1} images ...'.format(len(image_points_refine), len(image_points)))
    if len(image_points_refine) < 1:
        __logger.info('0 images for refinement available (adjust \"max_reprojection_error\"), exiting...'.format(len(image_points_refine), len(image_points)))
        sys.exit(1)
    result = calibrate_checker(checker_board, 
                               image_points_refine, 
                               object_points_refine, 
                               image_size, 
                               flags=flags,
                               use_releasing_object=use_releasing_object,
                               focal_length_mm=focal_length_mm,
                               pixel_size_mm=pixel_size_mm,
                               camera_matrix_init=camera_matrix_init,
                               dist_coeffs_init=dist_coeffs_init
                               )    
    return result 


def main():
    parser = argparse.ArgumentParser(formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    parser.add_argument('--config', help='path to config.jsonc', default='config_checker.jsonc')    
    args = parser.parse_args()

    if not os.path.exists(args.config):
        __logger.info('Config path {} does not exist:'.format(args.config))
        sys.exit(1)    

    cfg = configuration.load_config_json(args.config)
    input_dir = cfg['io_settings']['input_dir']
    output_dir = cfg['io_settings']['output_dir']
    img_extension = cfg['io_settings']['image_extension']

    # Create board
    checker_board = create_board_checker(**cfg['checker_board'])

    # Get image paths
    image_paths = glob.glob(os.path.join(input_dir, '*{}'.format(img_extension)))

    # Extract corners
    image_points, object_points, image_size, _ = extract_corners_checker(image_paths, checker_board, 
                                                                      **cfg['checker_corner_extraction_settings'],
                                                                      output_dir=output_dir)

    # Calibrate camera
    result = calibrate_checker(checker_board, image_points, object_points, image_size, 
                               flags=determine_checker_calibration_flag(cfg['checker_calibration_settings']['flags']),
                               use_releasing_object=cfg['checker_calibration_settings']['use_releasing_object'],
                               **cfg['camera_parameter'])

    # Refine calibration results
    result = calibrate_checker_refine(checker_board, image_points, object_points, image_size, 
                               flags=determine_checker_calibration_flag(cfg['checker_calibration_settings']['flags']),
                               use_releasing_object=cfg['checker_calibration_settings']['use_releasing_object'],
                               **cfg['camera_parameter'],                              
                               camera_matrix_init=result[1],
                               dist_coeffs_init=result[2],
                               initial_rep_errors=result[-1],
                               max_rep_err=cfg['checker_calibration_settings']['max_reprojection_error'])

    # Save result to xml
    camera_instrinsics_to_xml(result[1], result[2].reshape(-1), cfg['camera_parameter']['pixel_size_mm'], output_dir=output_dir)

    log.logging_config.move_log_file(__logger.handlers[0].baseFilename, os.path.join(output_dir, 'info.log'))


if __name__ == "__main__":
    np.set_printoptions(precision=6)
    main()

