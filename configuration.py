#!/usr/bin/env python

"""configuration.py: Contains functions for loading a config file as dict."""

import logging

import commentjson as json


def load_config_json(config_file, config_group=None):
    __logger = logging.getLogger(name=__name__)
    __logger.info(
        'Loading config file from this path: {}...'.format(config_file))
    config_dict = {}
    with open(config_file, 'r') as f:
        j_doc = json.load(f)

        # iterate over groups
        for group, group_dict in j_doc.items():
            group_dict = iterate_dict(group_dict)

            config_dict[group] = group_dict

    if config_group is not None:
        __logger.info('Loaded Config_Group: {}:\n{}'.format(
            config_group, json.dumps(config_dict[config_group], indent=4)))
        return config_dict[config_group]
    else:
        __logger.info('Loaded Config_Dict:\n{}'.format(
            json.dumps(config_dict, indent=4)))
        return config_dict


def iterate_dict(dictionary):
    for key, val in dictionary.items():
        if isinstance(val, dict):
            dictionary[key] = iterate_dict(dictionary[key])
        else:
            # set attributes with value 'None' to None
            if val == 'None':
                dictionary[key] = None
            if val == 'True':
                dictionary[key] = True
            if val == 'False':
                dictionary[key] = False
    return dictionary


if __name__ == '__main__':
    pass