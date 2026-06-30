#################################################################################
#   Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved.          #
#   Licensed under the Apache License, Version 2.0 (the "License").             #
#################################################################################
"""Model-metadata key constants (relocated out of the deleted ``boto`` package).

These are the keys of the ``model_metadata.json`` contract (sensor list, action
space, etc.). They used to live under ``deepracer_env.boto.s3.constants``; that
AWS/S3 package is removed in the ROS 2 port, but a couple of keys
(``steering_angle`` / ``speed``) are still used to label the per-step action, so
the enum lives here with no S3 coupling.
"""
from enum import Enum


class ModelMetadataKeys(Enum):
    """Keys used in the model-metadata JSON."""

    SENSOR = 'sensor'
    NEURAL_NETWORK = 'neural_network'
    VERSION = 'version'
    ACTION_SPACE = 'action_space'
    ACTION_SPACE_TYPE = 'action_space_type'
    TRAINING_ALGORITHM = 'training_algorithm'
    STEERING_ANGLE = 'steering_angle'
    SPEED = 'speed'
    LOW = 'low'
    HIGH = 'high'
    LIDAR_CONFIG = 'lidar_config'
    NUM_SECTORS = 'num_sectors'
    NUM_VALUES_PER_SECTOR = 'num_values_per_sector'
    CLIPPING_DISTANCE = 'clipping_dist'
