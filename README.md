# Camera-Calibration
This repository contains implementations of camera calibration algorithms based on OpenCV 4.2.0. Camera calibration can be done by using either a planar checkerboard pattern or a planar ChArUco pattern. Camera calibration for single cameras as well as extrinsic stereo camera calibration is supported for checker and ChArUco planar patterns.
## Requirements
#### Python = 3.6.8, OpenCV 4.2.0

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
In this repo, camera calibration is based on OpenCV 4.2.0 functionality. A `3x3 camera matrix with 4 parameters` (f<sub>x</sub>, f<sub>y</sub>, c<sub>x</sub>, c<sub>y</sub>) and `5 distortion parameters for tangential(p) and radial(k) distortions` (k<sub>1</sub>, k<sub>2</sub>, p<sub>1</sub>, p<sub>2</sub>, k<sub>3</sub>) are used. For more informations on this, see [here (OpenCV docu)](https://docs.opencv.org/4.2.0/d9/d0c/group__calib3d.html).
### Intrinsic Camera Calibration
For intrinsic camera calibraton (determining camera and distortion parameters) you can chose between two different methods.
Settings for calibration are provided by a json file inside the repository.
#### Checkerboard
1. Capture some images of your checkerboard with the camera you want to calibrate (`tip: for high-quality results, you'll need at least ten images of a 7-by-8 or larger chessboard (and that's only if you move the chessboard enough between images to obtain a "rich" set of views`)
2. Setup the configuration file by opening: `path/to/Camera-Calibration/calib_checker.jsonc`
3. Start script:

        $ cd path/to/Camera-Calibration/
        $ activate camcalib
        $ python calib_checker.py

#### ChArUco

### Extrinsic Stereo Camera Calibration

#### Checkerboard

#### ChArUco

## Logging
- __info.log__ which logs what is happening and will be copied to your output directory after the script is done or stay in `path/to/Camera-Calibration/log/` if something causes the script to crash



