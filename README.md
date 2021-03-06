# Camera-Calibration
This repository contains implementations of camera calibration algorithms based on OpenCV 4.2.0. Camera calibration can be done by using either a planar checkerboard pattern or a planar ChArUco pattern. Camera calibration for single cameras as well as extrinsic stereo camera calibration is supported for checker and ChArUco planar patterns.
## Requirements
#### Python 3.6.8, OpenCV 4.2.0

#### Installation
Using [conda](https://docs.conda.io/en/latest/miniconda.html) for managing virtual environments.

    $ git clone https://github.com/RichLin56/Camera-Calibration.git
    $ cd path/to/Camera-Calibration/
    $ conda create -n camcalib python==3.6.8
    $ conda activate camcalib
    $ conda install -c conda-forge opencv==4.2.0
    $ conda install -c conda-forge matplotlib==3.2.1
    $ pip install commentjson==0.8.3

## Camera Calibration
In this repo, camera calibration is based on OpenCV 4.2.0 functionality. A `3x3 camera matrix with 4 parameters` (__f<sub>x</sub>, f<sub>y</sub>, c<sub>x</sub>, c<sub>y</sub>__) and `5 distortion parameters for tangential(p) and radial(k) distortions` (__k<sub>1</sub>, k<sub>2</sub>, p<sub>1</sub>, p<sub>2</sub>, k<sub>3</sub>__) are used. For more informations on this, see [here (OpenCV docu)](https://docs.opencv.org/4.2.0/d9/d0c/group__calib3d.html).
### Intrinsic Camera Calibration
For intrinsic camera calibraton (determining camera and distortion parameters) you can chose between two different planar targets.
Settings for calibration are provided by a .json file inside the repository.
#### Settings in config
1. `io_settings`: Paths to input and output directory
2. `board_settings`: Information about the board used for camera calibration
3. `camera_parameter_settings`: Information about the camera which is going to be calibrated
4. `corner_extraction_settings`: Option to choose between different algorithms and set their arguments(flags). Explanation at the bottom of config file
5. `calibration_settings`: Option to choose between different algorithms and set their arguments(flags). Explanation at the bottom of config file
#### General Pipeline in Scripts
 1. Load settings from config
 2. Create board
 3. Read image paths
 4. Extract corners in images
 5. Calibrate camera
 6. Refine calibration results by using only images which resulted in a max. reprojection error lower than what was specified in the config
#### Output Directory Structure
The output directory contains the `camera_intrinsics.xml` file in which the calibration parameters are stored. Additionally, a visualization of the detected corners and a log file is also be stored in the output directory.

        output_dir           # path:  /path/to/output_dir
         ├── CornerVisualization         
         |    └──corners_image_1.extension
         |    └──corners_image_1.extension
         |    └──...
         ├── info.log
         └── camera_intrinsics.xml
         
#### Checkerboard - calib_checker.py
 1. Capture some images of your checkerboard with the camera you want to calibrate (`tip: for high-quality results, you'll need at least ten images of a 7-by-8 or larger chessboard (and that's only if you move the chessboard enough between images to obtain a "rich" set of views`)
 2. Setup the configuration file by opening: `path/to/Camera-Calibration/calib_checker.jsonc`
 3. Start script:

        $ cd path/to/Camera-Calibration/
        $ activate camcalib
        $ python calib_checker.py
        
 4. Check script output which will be saved in `path/to/output_dir`(configured in config.jsonc)
 5. (Optional) restart script with improved settings

#### ChArUco - calib_charuco.py
 1. Capture some images of your ChArUcoboard with the camera you want to calibrate (`tip: for high-quality results, you'll need at least ten images of a 7-by-8 or larger ChArUcoboard (and that's only if you move the ChArUcoboard enough between images to obtain a "rich" set of views`)
 2. Setup the configuration file by opening: `path/to/Camera-Calibration/calib_charuco.jsonc`
 3. Start script:

        $ cd path/to/Camera-Calibration/
        $ activate camcalib
        $ python calib_charuco.py
        
 4. Check script output which will be saved in `path/to/output_dir`(configured in config.jsonc)
 5. (Optional) restart script with improved settings

### Extrinsic Stereo Camera Calibration 
For extrinsic stereo camera calibraton (determining the position of the cameras relative to each other) you can chose between two different planar targets. Settings for calibration are provided by a .json file inside the repository.
#### Settings in config
1. `io_settings`: Paths to input, output directory and camera_intrinsics.xml for left and right camera in your stereo camera rig
2. `board_settings`: Information about the board used for stereo camera calibration
3. `corner_extraction_settings`: Option to choose between different algorithms and set their arguments(flags). Explanation at the bottom of config file
4. `calibration_settings`: Option to choose between different algorithms and set their arguments(flags). Explanation at the bottom of config file
#### Output Directory Structure
The output directory contains the `stereo_camera_extrinsics.xml` file in which the calibration parameters are stored. Additionally, a visualization of the detected corners and a log file is also be stored in the output directory.

        output_dir           # path:  /path/to/output_dir
         ├── CornerVisualization         
         |    └──corners_image_1.extension
         |    └──corners_image_1.extension
         |    └──...
         ├── info.log
         └── camera_extrinsics.xml
 
 #### Input Directory Structure
`Attention`: Make sure while capturing images with your stereo rig, that __`images belonging to the left camera have "left" in their filenames and images beloging to the right camera have "right" in their filenames`__. __`Use a time stemp in the filename or a unique ID for every pair`__, since images will be sorted by name, then split into left and right groups and paired afterwards based on their position in the group (see below for example).
 
 #### This works:
          input_dir           # path:  /path/to/input_dir
                 |      
                 └──2020_05_21_15_18_2129_left.extension
                 └──2020_05_21_15_18_2150_right.extension
                 └──2020_05_21_15_24_2200_left.extension
                 └──2020_05_21_15_24_2255_right.extension
                 └──... 
 #### This works as well: 
           input_dir           # path:  /path/to/input_dir
                 |      
                 └──left_1.extension
                 └──left_2.extension
                 └──right_1.extension 
                 └──right_2.extension
                 └──... 

#### Checkerboard - calib_checker_stereo.py
 1. Capture some images of your checkerboard with the stereo camera rig you want to calibrate (`tip: for high-quality results, you'll need at least ten images of a 7-by-8 or larger chessboard (and that's only if you move the chessboard enough between images to obtain a "rich" set of views`). 
                 
 2. Make sure you've made an intrinsic calibration for each of your cameras.
 3. Setup the configuration file by opening: `path/to/Camera-Calibration/calib_checker_stereo.jsonc`
 4. Start script:

        $ cd path/to/Camera-Calibration/
        $ activate camcalib
        $ python calib_checker_stereo.py
        
 5. Check script output which will be saved in `path/to/output_dir`(configured in config.jsonc)
 6. (Optional) restart script with improved settings

#### ChArUco - calib_charuco_stereo.py
 1. Capture some images of your ChArUcoboard with the stereo camera rig you want to calibrate (`tip: for high-quality results, you'll need at least ten images of a 7-by-8 or larger ChArUcoboard (and that's only if you move the ChArUcoboard enough between images to obtain a "rich" set of views`)
 2. Make sure you've made an intrinsic calibration for each of your cameras.
 3. Setup the configuration file by opening: `path/to/Camera-Calibration/calib_charuco_stereo.jsonc`
 4. Start script:

        $ cd path/to/Camera-Calibration/
        $ activate camcalib
        $ python calib_charuco_stereo.py
        
 5. Check script output which will be saved in `path/to/output_dir`(configured in config.jsonc)
 6. (Optional) restart script with improved settings

## Logging
- __info.log__ which logs what is happening and will be copied to your output directory after the script is done or stay in `path/to/Camera-Calibration/log/` if something causes the script to crash

## To Do
 - [ ] calib_checker.py - line 202: Implement option (in config-file) to display these values in log-file
 - [ ] calib_charuco.py - line 149: Implement option (in config-file) to display these values in log-file
 - [ ] calib_checker_stereo.py - line 160: Implement refinement based on max_rep_error (when time is available)
 - [ ] calib_charuco_stereo.py - line 138: Median method is good, but i think it is better to use median as starting point for some optimization method
 - [ ] Implement parameters for the ChArUco detectMarker process

