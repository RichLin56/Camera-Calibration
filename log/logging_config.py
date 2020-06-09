#!/usr/bin/env python

"""logging_config.py: Contains functions related to logging."""

__author__ = "Richard Lindenpuetz"
__email__ = "richard.lindenpuetz@rwth-aachen.de"
__license__ = "MIT"
__version__ = "1.0.0"

import os
import logging
import logging.config
import shutil
from datetime import datetime

import commentjson as json


def setup_logging(
        path_to_config,
        default_level=logging.INFO,
        log_dir=None):
    """Setup logging configuration.
    """
    if os.path.exists(path_to_config):
        with open(path_to_config, 'rt') as f:
            config = json.load(f)
            if log_dir is None:
                log_path = os.path.join(os.path.dirname(path_to_config), '{}_info.log'.format(
                    datetime.now().strftime("%Y-%m-%d_%I-%M-%S_%p")))
            else:
                log_path = os.path.join(log_dir, 'info.log')
                os.makedirs(log_dir, exist_ok=True)
            config['handlers']['info_file_handler']['filename'] = log_path
            logging.config.dictConfig(config)
    else:
        logging.basicConfig(level=default_level)
        log_path = None
    return log_path


def get_logger(name=__name__):
    return logging.getLogger(name)


def shut_down_all_logger():
    logging.shutdown()


def move_log_file(src, dst):
    shut_down_all_logger()
    shutil.move(src, dst)


if __name__ == '__main__':
    pass

"""
@history
__version__ = "1.0.0" -> basic functionality
"""
