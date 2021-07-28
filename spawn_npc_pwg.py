#!/usr/bin/env python3.7

# Copyright (c) 2019 Computer Vision Center (CVC) at the Universitat Autonoma de
# Barcelona (UAB).
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

"""Spawn NPCs into the simulation"""

from __future__ import print_function
from math import sqrt
import glob
import os
import sys
import time
import random
from agents.navigation.basic_agent import BasicAgent

try:
    sys.path.append(glob.glob('../carla/dist/carla-*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass

import carla

from carla import VehicleLightState as vls

import argparse
import logging

ZONE_1 = [[-20, 0, 0], [-20, -5, 0], [-18, 10,  0], [15, 15, 0], [-5, 20, 0], [-5, 23, 0], [-20, 15, 0], [-15, 20, 0], [-20, 8, 0], [-24, 0, 0], [-24, 5, 0], [-12, 25,0], [-8, 25, 0], [-11, 30, 0],
          [-7, 30, 0], [-10, 35, 0], [-7, 35, 0], [-10, 40, 0], [-10, 45, 0], [-10, 50, 0], [-10, 55, 0], [-10, 60, 0], [-10, 65, 0], [-10, 70, 0], [-7, 40, 0], [-7, 45, 0], [-7, 50, 0], [-7, 55, 0],
          [-7, 60, 0], [-7, 65, 0], [-7, 70, 0], [-10, 75, 0], [-10, 80, 0], [-10, 85, 0], [-10, 90, 0], [-10, 95, 0], [-10, 100, 0], [-10, 105, 0], [-10, 110, 0], [-10, 115, 0], [-10, 120, 0],
          [-10, 125, 0], [-10, 130, 0], [-10, 135, 0], [-10, 140, 0],   [-10, 150, 0],  [-7, 75, 0], [-7, 80, 0], [-7, 85, 0], [-7, 90, 0], [-7, 95, 0], [-7, 100, 0], [-7, 105, 0],  [-6, 110, 0],
          [-6, 115, 0], [-6, 120, 0], [-6, 125, 0], [-6, 130, 0], [-6, 135, 0], [-6, 140, 0],   [-6, 150, 0], [-6, 135, 0], [-10, 135, 0], [-9, 155, 0], [-9, 160, 0], [-9, 165, 0], [-9, 170, 0],
          [-9, 175, 0], [-9, 180, 0], [-9, 185, 0],  [-9, 190, 0],  [-5, 155, 0], [-5, 160, 0], [-5, 165, 0], [-5, 170, 0], [-5, 175, 0], [-5, 180, 0], [-5, 185, 0],  [-5, 190, 0], [-5, 193, 0],
          [-5, 197, 0], [-5, 204, 0], [-5, 207, 0], [-10, 193, 0], [-10, 197, 0], [-10, 204, 0], [-10, 207, 0], [-15, 193, 0], [-15, 197, 0], [-15, 204, 0], [-15, 207, 0], [-20, 193, 0],
          [-20, 197, 0], [-20, 204, 0], [-20, 207, 0], [-25, 193, 0], [-25, 197, 0], [-25, 204, 0], [-25, 207, 0], [-30, 193, 0], [-30, 197, 0], [-30, 204, 0], [-30, 207, 0], [-35, 193, 0],
          [-35, 197, 0], [-35, 204, 0], [-35, 207, 0], [-40, 193, 0], [-40, 197, 0], [-40, 204, 0], [-40, 207, 0], [-45, 193, 0], [-45, 197, 0], [-45, 204, 0], [-45, 207, 0], [-40, 1, 0], [-45, 1, 0],
          [-50, 1, 0], [-55, 1, 0], [-60, 1, 0], [-65, 1, 0], [-74, 0, 0], [-74, 5, 0], [-74, 10, 0], [-74, 15, 0], [-74, 20, 0], [-74, 25, 0], [-74, 30, 0], [-74, 35, 0], [-74, 40, 0], [-74, 45, 0],
          [-74, 50, 0], [-78, 0, 0], [-78, 5, 0], [-78, 10, 0], [-78, 15, 0], [-78, 20, 0], [-78, 25, 0], [-78, 30, 0], [-78, 35, 0], [-78, 40, 0], [-78, 45, 0], [-78, 50, 0], [-81, 0, 0],
          [-81, 5, 0], [-81, 10, 0], [-81, 15, 0], [-81, 20, 0], [-81, 25, 0], [-81, 30, 0], [-81, 35, 0], [-81, 40, 0], [-81, 45, 0], [-81, 50, 0], [-85, 0, 0], [-85, 5, 0], [-85, 10, 0],
          [-85, 15, 0], [-85, 20, 0], [-85, 25, 0], [-85, 30, 0], [-85, 35, 0], [-85, 40, 0], [-85, 45, 0], [-85, 50, 0], [-88, 0, 0], [-88, 5, 0], [-88, 10, 0], [-88, 15, 0], [-88, 20, 0],
          [-88, 25, 0], [-88, 30, 0], [-88, 35, 0], [-88, 40, 0], [-88, 45, 0], [-88, 50, 0],  [-74, 55, 0], [-74, 60, 0], [-74, 65, 0], [-74, 70, 0], [-74, 75, 0], [-74, 80, 0], [-74, 85, 0],
          [-74, 90, 0], [-74, 95, 0], [-74, 100, 0], [-74, 105, 0], [-74, 110, 0], [-74, 115, 0], [-74, 120, 0], [-78, 55, 0], [-78, 60, 0], [-78, 65, 0], [-78, 70, 0], [-78, 75, 0], [-78, 80, 0],
          [-78, 85, 0], [-78, 90, 0], [-78, 95, 0], [-78, 100, 0], [-78, 105, 0], [-78, 110, 0], [-78, 115, 0], [-78, 120, 0], [-81, 55, 0], [-81, 60, 0], [-81, 65, 0], [-81, 70, 0], [-81, 75, 0],
          [-81, 80, 0], [-81, 85, 0], [-81, 90, 0], [-81, 95, 0], [-81, 100, 0], [-81, 105, 0], [-81, 110, 0], [-81, 115, 0], [-81, 120, 0], [-85, 55, 0], [-85, 60, 0], [-85, 65, 0], [-85, 70, 0],
          [-85, 75, 0], [-85, 80, 0], [-85, 85, 0], [-85, 90, 0], [-85, 95, 0], [-85, 100, 0], [-85, 105, 0], [-85, 110, 0], [-85, 115, 0], [-85, 120, 0], [-88, 55, 0], [-88, 60, 0], [-88, 65, 0],
          [-88, 70, 0], [-88, 75, 0], [-88, 80, 0], [-88, 85, 0], [-88, 90, 0], [-88, 95, 0], [-88, 100, 0], [-88, 105, 0], [-88, 110, 0], [-88, 115, 0], [-88, 120, 0], [-74, 125, 0], [-74, 130, 0],
          [-74, 135, 0], [-74, 140, 0], [-74, 145, 0], [-74, 150, 0], [-74, 155, 0], [-74, 60, 0], [-78, 125, 0], [-78, 130, 0], [-78, 135, 0], [-78, 140, 0], [-78, 145, 0], [-78, 150, 0],
          [-78, 155, 0], [-78, 60, 0], [-81, 125, 0], [-81, 130, 0], [-81, 135, 0], [-81, 140, 0], [-81, 145, 0], [-81, 150, 0], [-81, 155, 0], [-81, 60, 0], [-85, 125, 0], [-85, 130, 0],
          [-85, 135, 0], [-85, 140, 0], [-85, 145, 0], [-85, 150, 0], [-85, 155, 0], [-85, 60, 0], [-88, 125, 0], [-88, 130, 0], [-88, 135, 0], [-88, 140, 0], [-88, 145, 0], [-88, 150, 0],
          [-88, 155, 0], [-88, 60, 0], [-90, 140, 0], [-95, 140, 0], [-100, 140, 0], [-105, 140, 0], [-110, 140, 0], [-115, 140, 0], [-120, 140, 0], [-125, 140, 0], [-90, 137, 0], [-95, 137, 0],
          [-100, 137, 0], [-105, 137, 0], [-110, 137, 0], [-115, 137, 0], [-120, 137, 0], [-125, 137, 0], [-90, 137, 0], [-95, 137, 0], [-100, 137, 0], [-105, 137, 0], [-110, 137, 0], [-115, 137, 0],
          [-120, 137, 0], [-125, 137, 0], [-90, 133, 0], [-95, 133, 0], [-100, 133, 0], [-105, 133, 0], [-110, 133, 0], [-115, 133, 0], [-120, 133, 0], [-125, 133, 0], [-90, 130, 0], [-95, 130, 0],
          [-100, 130, 0], [-105, 130, 0], [-110, 130, 0], [-115, 130, 0], [-120, 130, 0], [-125, 130, 0], [-20, 142, 0], [-25, 142, 0], [-30, 142, 0], [-35, 142, 0], [-40, 142, 0], [-45, 142, 0],
          [-50, 142, 0], [-55, 142, 0], [-60, 142, 0], [-65, 142, 0], [-70, 142, 0], [-75, 142, 0], [-80, 142, 0], [-85, 142, 0], [-90, 142, 0], [-20, 135, 0], [-25, 135, 0], [-30, 135, 0],
          [-35, 135, 0], [-40, 135, 0], [-45, 135, 0], [-50, 135, 0], [-55, 135, 0], [-60, 135, 0], [-65, 135, 0], [-70, 135, 0], [-75, 135, 0], [-80, 135, 0], [-85, 135, 0], [-90, 135, 0],
          [-20, 131, 0], [-25, 131, 0], [-30, 131, 0], [-35, 131, 0], [-40, 131, 0], [-45, 131, 0], [-50, 131, 0], [-55, 131, 0], [-60, 131, 0], [-65, 131, 0], [-70, 131, 0], [-75, 131, 0],
          [-80, 131, 0], [-85, 131, 0], [-90, 131, 0], [-20, 127.5, 0], [-25, 127.5, 0], [-30, 127.5, 0], [-35, 127.5, 0], [-40, 127.5, 0], [-45, 127.5, 0], [-50, 127.5, 0], [-55, 127.5, 0],
          [-60, 127.5, 0], [-65, 127.5, 0], [-70, 127.5, 0], [-75, 127.5, 0], [-80, 127.5, 0], [-85, 127.5, 0], [-90, 127.5, 0]]

ZONE_1_BOUNDARIES = [[0, 0, 0], [-158, 0, 0], [0, 212, 0], [-158, 212, 0]]


ZONE_2 = [[-88, -125, 0], [-88, -120, 0], [-88, -115, 0], [-88, -110, 0], [-88, -105, 0], [-88, -100, 0], [-88, -95, 0], [-88, -90, 0], [-88, -85, 0], [-88, -80, 0], [-88, -75, 0], [-88, -70, 0],
          [-88, -65, 0], [-88, -60, 0], [-88, -55, 0], [-88, -50, 0], [-88, -45, 0], [-88, -40, 0], [-88, -35, 0], [-88, -30, 0], [-88, -25, 0], [-88, -20, 0], [-88, -15, 0], [-88, -10, 0],
          [-85, -125, 0], [-85, -120, 0], [-85, -115, 0], [-85, -110, 0], [-85, -105, 0], [-85, -100, 0], [-85, -95, 0], [-85, -90, 0], [-85, -85, 0], [-85, -80, 0], [-85, -75, 0], [-85, -70, 0],
          [-85, -65, 0], [-85, -60, 0], [-85, -55, 0], [-85, -50, 0], [-85, -45, 0], [-85, -40, 0], [-85, -35, 0], [-85, -30, 0], [-85, -25, 0], [-85, -20, 0], [-85, -15, 0], [-85, -10, 0],
          [-82, -125, 0], [-82, -120, 0], [-82, -115, 0], [-82, -110, 0], [-82, -105, 0], [-82, -100, 0], [-82, -95, 0], [-82, -90, 0], [-82, -85, 0], [-82, -80, 0], [-82, -75, 0], [-82, -70, 0],
          [-82, -65, 0], [-82, -60, 0], [-82, -55, 0], [-82, -50, 0], [-82, -45, 0], [-82, -40, 0], [-82, -35, 0], [-82, -30, 0], [-82, -25, 0], [-82, -20, 0], [-82, -15, 0], [-82, -10, 0],
          [-78, -125, 0], [-78, -120, 0], [-78, -115, 0], [-78, -110, 0], [-78, -105, 0], [-78, -100, 0], [-78, -95, 0], [-78, -90, 0], [-78, -85, 0], [-78, -80, 0], [-78, -75, 0], [-78, -70, 0],
          [-78, -65, 0], [-78, -60, 0], [-78, -55, 0], [-78, -50, 0], [-78, -45, 0], [-78, -40, 0], [-78, -35, 0], [-78, -30, 0], [-78, -25, 0], [-78, -20, 0], [-78, -15, 0], [-78, -10, 0],
          [-75, -125, 0], [-75, -120, 0], [-75, -115, 0], [-75, -110, 0], [-75, -105, 0], [-75, -100, 0], [-75, -95, 0], [-75, -90, 0], [-75, -85, 0], [-75, -80, 0], [-75, -75, 0], [-75, -70, 0],
          [-75, -65, 0], [-75, -60, 0], [-75, -55, 0], [-75, -50, 0], [-75, -45, 0], [-75, -40, 0], [-75, -35, 0], [-75, -30, 0], [-75, -25, 0], [-75, -20, 0], [-75, -15, 0], [-75, -10, 0],
          [-15, -142, 0], [-20, -142, 0], [-25, -142, 0], [-30, -142, 0], [-35, -143,  0], [-40, -143,  0], [-45, -143,  0], [-50, -143,  0], [-55, -143,  0], [-60, -143,  0], [-65, -143,  0],
          [-15, -139, 0], [-20, -149, 0], [-25, -139, 0], [-30, -139, 0], [-35, -140,  0], [-40, -140,  0], [-45, -140,  0], [-50, -140,  0], [-55, -140,  0], [-60, -140,  0], [-65, -140,  0],
          [-15, -135, 0], [-20, -135, 0], [-25, -135, 0], [-30, -135, 0], [-35, -136,  0], [-40, -136,  0], [-45, -136,  0], [-50, -136,  0], [-55, -136,  0], [-60, -136,  0], [-65, -136,  0],
          [-15, -132, 0], [-20, -132, 0], [-25, -132, 0], [-30, -132, 0], [-35, -133,  0], [-40, -133,  0], [-45, -133,  0], [-50, -133,  0], [-55, -133,  0], [-60, -133,  0], [-65, -133,  0],
          [-15, -195, 0], [-20, -195, 0], [-25, -195, 0], [-30, -195, 0], [-35, -195, 0], [-40, -195, 0], [-45, -195, 0], [-50, -195, 0], [-55, -195, 0], [-15, -198, 0], [-20, -198, 0],
          [-25, -198, 0], [-30, -198, 0], [-35, -198, 0], [-40, -198, 0], [-45, -198, 0], [-50, -198, 0], [-55, -198, 0], [-15, -202, 0], [-20, -202, 0], [-25, -202, 0], [-30, -202, 0],
          [-35, -202, 0], [-40, -202, 0], [-45, -202, 0], [-50, -202, 0], [-55, -202, 0], [-15, -205, 0], [-20, -205, 0], [-25, -205, 0], [-30, -205, 0], [-35, -205, 0], [-40, -205, 0],
          [-45, -205, 0], [-50, -205, 0], [-55, -205, 0], [-15, -208, 0], [-20, -208, 0], [-25, -208, 0], [-30, -208, 0], [-35, -208, 0], [-40, -208, 0], [-45, -208, 0], [-50, -208, 0], [-55, -208, 0]]

ZONE_2_BOUNDARIES = [[0, 0, 0], [-158, 0, 0], [0, -212, 0], [-158, -212, 0]]

ZONE_3 = [[10, 193, 0], [15, 193, 0], [20, 193, 0], [25, 193, 0], [30, 193, 0], [35, 193, 0], [40, 193, 0], [45, 193, 0], [50, 193, 0], [55, 193, 0], [60, 193, 0], [65, 193, 0], [70, 193, 0],
          [75, 193, 0], [80, 193, 0], [85, 193, 0], [90, 193, 0], [95, 193, 0], [100, 193, 0], [105, 193, 0], [110, 193, 0], [105, 193, 0], [110, 193, 0], [115, 193, 0], [120, 193, 0], [125, 193, 0],
          [130, 193, 0], [135, 193, 0], [140, 193, 0], [145, 193, 0], [150, 193, 0], [155, 193, 0], [160, 193, 0], [10, 197, 0], [15, 197, 0], [20, 197, 0], [25, 197, 0], [30, 197, 0], [35, 197, 0],
          [40, 197, 0], [45, 197, 0], [50, 197, 0], [55, 197, 0], [60, 197, 0], [65, 197, 0], [70, 197, 0], [75, 197, 0], [80, 197, 0], [85, 197, 0], [90, 197, 0], [95, 197, 0], [100, 197, 0],
          [105, 197, 0], [110, 197, 0], [105, 197, 0], [110, 197, 0], [115, 197, 0], [120, 197, 0], [125, 197, 0], [130, 197, 0], [135, 197, 0], [140, 197, 0], [145, 197, 0], [150, 197, 0],
          [155, 197, 0], [160, 197, 0], [10, 200, 0], [15, 200, 0], [20, 200, 0], [25, 200, 0], [30, 200, 0], [35, 200, 0], [40, 200, 0], [45, 200, 0], [50, 200, 0], [55, 200, 0], [60, 200, 0],
          [65, 200, 0], [70, 200, 0], [75, 200, 0], [80, 200, 0], [85, 200, 0], [90, 200, 0], [95, 200, 0], [100, 200, 0], [105, 200, 0], [110, 200, 0], [105, 200, 0], [110, 200, 0], [115, 200, 0],
          [120, 200, 0], [125, 200, 0], [130, 200, 0], [135, 200, 0], [140, 200, 0], [145, 200, 0], [150, 200, 0], [155, 200, 0], [160, 200, 0], [10, 204, 0], [15, 204, 0], [20, 204, 0], [25, 204, 0],
          [30, 204, 0], [35, 204, 0], [40, 204, 0], [45, 204, 0], [50, 204, 0], [55, 204, 0], [60, 204, 0], [65, 204, 0], [70, 204, 0], [75, 204, 0], [80, 204, 0], [85, 204, 0], [90, 204, 0],
          [95, 204, 0], [100, 204, 0], [105, 204, 0], [110, 204, 0], [105, 204, 0], [110, 204, 0], [115, 204, 0], [120, 204, 0], [125, 204, 0], [130, 204, 0], [135, 204, 0], [140, 204, 0],
          [145, 204, 0], [150, 204, 0], [155, 204, 0], [160, 204, 0], [10, 207.5, 0], [15, 207.5, 0], [20, 207.5, 0], [25, 207.5, 0], [30, 207.5, 0], [35, 207.5, 0], [40, 207.5, 0], [45, 207.5, 0],
          [50, 207.5, 0], [55, 207.5, 0], [60, 207.5, 0], [65, 207.5, 0], [70, 207.5, 0], [75, 207.5, 0], [80, 207.5, 0], [85, 207.5, 0], [90, 207.5, 0], [95, 207.5, 0], [100, 207.5, 0],
          [105, 207.5, 0], [110, 207.5, 0], [105, 207.5, 0], [110, 207.5, 0], [115, 207.5, 0], [120, 207.5, 0], [125, 207.5, 0], [130, 207.5, 0], [135, 207.5, 0], [140, 207.5, 0], [145, 207.5, 0],
          [150, 207.5, 0], [155, 207.5, 0], [160, 207.5, 0], [25, -193, 0], [30, -193, 0], [35, -193, 0], [40, -193, 0], [45, -193, 0], [50, -193, 0], [55, -193, 0], [60, -193, 0], [65, -192, 0],
          [70, -192, 0], [75, -192, 0], [80, -192, 0], [85, -192, 0], [90, -192, 0], [95, -192, 0], [100, -192, 0], [105, -191, 0], [110, -191, 0], [115, -191, 0], [120, -191, 0], [125, -191, 0],
          [130, -191, 0], [135, -191, 0], [140, -191, 0], [25, -195, 0], [30, -195, 0], [35, -195, 0], [40, -195, 0], [45, -195, 0], [50, -195, 0], [55, -195, 0], [60, -195, 0], [65, -194, 0],
          [70, -194, 0], [75, -194, 0], [80, -194, 0], [85, -194, 0], [90, -194, 0], [95, -194, 0], [100, -194, 0], [105, -193, 0], [110, -193, 0], [115, -193, 0], [120, -193, 0], [125, -193, 0],
          [130, -193, 0], [135, -193, 0], [140, -193, 0], [25, -199, 0], [30, -199, 0], [35, -199, 0], [40, -199, 0], [45, -199, 0], [50, -199, 0], [55, -199, 0], [60, -199, 0], [65, -198, 0],
          [70, -198, 0], [75, -198, 0], [80, -198, 0], [85, -198, 0], [90, -198, 0], [95, -198, 0], [100, -198, 0], [105, -197, 0], [110, -197, 0], [115, -197, 0], [120, -197, 0], [125, -197, 0],
          [130, -197, 0], [135, -197, 0], [140, -197, 0], [25, -203, 0], [30, -203, 0], [35, -203, 0], [40, -203, 0], [45, -203, 0], [50, -203, 0], [55, -203, 0], [60, -203, 0], [65, -202, 0],
          [70, -202, 0], [75, -202, 0], [80, -202, 0], [85, -202, 0], [90, -202, 0], [95, -202, 0], [100, -202, 0], [105, -201, 0], [110, -201, 0], [115, -201, 0], [120, -201, 0], [125, -201, 0],
          [130, -201, 0], [135, -201, 0], [140, -201, 0], [25, -205, 0], [30, -205, 0], [35, -205, 0], [40, -205, 0], [45, -205, 0], [50, -205, 0], [55, -205, 0], [60, -205, 0], [65, -204, 0],
          [70, -204, 0], [75, -204, 0], [80, -204, 0], [85, -204, 0], [90, -204, 0], [95, -204, 0], [100, -204, 0], [105, -203, 0], [110, -203, 0], [115, -203, 0], [120, -203, 0], [125, -203, 0],
          [130, -203, 0], [135, -203, 0], [140, -203, 0]]

ZONE_3_BOUNDARIES = [[0, 212, 0], [0, -212, 0], [255, 212, 0], [255, -212, 0]]

ZONES = [ZONE_1, ZONE_2, ZONE_3]


def zone_name(zone):
    if (zone == ZONE_1):
        return "Zone 1"
    elif (zone == ZONE_2):
        return "Zone 2"
    else:
        return "Zone 3"

    
def next_location(actor_agent, prev_location):
    next_zone = random.choice(ZONES)
    next_waypoint = random.choice(next_zone)
    actor_agent.set_destination(next_waypoint)
    distance = sqrt(   (next_waypoint[0] - prev_location[0])**2 + (next_waypoint[1] - prev_location[1])**2      ) 
    print("Next Zone:" , zone_name(next_zone), "\n", "Next Location: ", next_waypoint, "\n", "Distance: ", distance)
    return next_waypoint


def main():
    argparser = argparse.ArgumentParser(
        description=__doc__)
    argparser.add_argument(
        '--host',
        metavar='H',
        default='127.0.0.1',
        help='IP of the host server (default: 127.0.0.1)')
    argparser.add_argument(
        '-p', '--port',
        metavar='P',
        default=2000,
        type=int,
        help='TCP port to listen to (default: 2000)')
    argparser.add_argument(
        '-n', '--number-of-vehicles',
        metavar='N',
        default=1,
        type=int,
        help='number of vehicles (default: 1)')
    argparser.add_argument(
        '-w', '--number-of-walkers',
        metavar='W',
        default=50,
        type=int,
        help='number of walkers (default: 50)')
    argparser.add_argument(
        '--safe',
        action='store_true',
        help='avoid spawning vehicles prone to accidents')
    argparser.add_argument(
        '--filterv',
        metavar='PATTERN',
        default='vehicle.*',
        help='vehicles filter (default: "vehicle.*")')
    argparser.add_argument(
        '--filterw',
        metavar='PATTERN',
        default='walker.pedestrian.*',
        help='pedestrians filter (default: "walker.pedestrian.*")')
    argparser.add_argument(
        '--tm-port',
        metavar='P',
        default=8000,
        type=int,
        help='port to communicate with TM (default: 8000)')
    argparser.add_argument(
        '--sync',
        action='store_true',
        help='Synchronous mode execution')
    argparser.add_argument(
        '--hybrid',
        action='store_true',
        help='Enanble')
    argparser.add_argument(
        '-s', '--seed',
        metavar='S',
        type=int,
        help='Random device seed')
    argparser.add_argument(
        '--car-lights-on',
        action='store_true',
        default=False,
        help='Enanble car lights')
    args = argparser.parse_args()

    logging.basicConfig(format='%(levelname)s: %(message)s', level=logging.INFO)

    vehicles_list = []
    walkers_list = []
    all_id = []
    client = carla.Client(args.host, args.port)
    client.set_timeout(10.0)
    synchronous_master = False
    random.seed(args.seed if args.seed is not None else int(time.time()))

    try:
        world = client.get_world()

        traffic_manager = client.get_trafficmanager(args.tm_port)
        traffic_manager.set_global_distance_to_leading_vehicle(1.0)
        if args.hybrid:
            traffic_manager.set_hybrid_physics_mode(True)
        if args.seed is not None:
            traffic_manager.set_random_device_seed(args.seed)


        if args.sync:
            settings = world.get_settings()
            traffic_manager.set_synchronous_mode(True)
            if not settings.synchronous_mode:
                synchronous_master = True
                settings.synchronous_mode = True
                settings.fixed_delta_seconds = 0.05
                world.apply_settings(settings)
            else:
                synchronous_master = False

        blueprints = world.get_blueprint_library().filter(args.filterv)
        blueprintsWalkers = world.get_blueprint_library().filter(args.filterw)

        if args.safe:
            blueprints = [x for x in blueprints if int(x.get_attribute('number_of_wheels')) == 4]
            blueprints = [x for x in blueprints if not x.id.endswith('isetta')]
            blueprints = [x for x in blueprints if not x.id.endswith('carlacola')]
            blueprints = [x for x in blueprints if not x.id.endswith('cybertruck')]
            blueprints = [x for x in blueprints if not x.id.endswith('t2')]

        blueprints = sorted(blueprints, key=lambda bp: bp.id)

        spawn_points = world.get_map().get_spawn_points()
        number_of_spawn_points = len(spawn_points)

        if args.number_of_vehicles < number_of_spawn_points:
            random.shuffle(spawn_points)
        elif args.number_of_vehicles > number_of_spawn_points:
            msg = 'requested %d vehicles, but could only find %d spawn points'
            logging.warning(msg, args.number_of_vehicles, number_of_spawn_points)
            args.number_of_vehicles = number_of_spawn_points

        # @todo cannot import these directly.
        SpawnActor = carla.command.SpawnActor
        SetAutopilot = carla.command.SetAutopilot
        SetVehicleLightState = carla.command.SetVehicleLightState
        FutureActor = carla.command.FutureActor







        
        # --------------
        # Spawn vehicles
        # --------------
        actor_list = []
        for i in range(args.number_of_vehicles):
            if i >= args.number_of_vehicles:
                break

            try:
                #client = carla.Client("localhost", 2000)
                #client.set_timeout(2.0)
                #world = client.get_world()

                blueprint = random.choice(blueprints)
                if blueprint.has_attribute('color'):
                    color = random.choice(blueprint.get_attribute('color').recommended_values)
                    blueprint.set_attribute('color', color)
                if blueprint.has_attribute('driver_id'):
                    driver_id = random.choice(blueprint.get_attribute('driver_id').recommended_values)
                    blueprint.set_attribute('driver_id', driver_id)




                # prepare the light state of the cars to spawn
                #light_state = vls.NONE
                #if args.car_lights_on:
                #light_state = vls.Position | vls.LowBeam | vls.LowBeam
                # spawn the cars and set their autopilot and light state all together
                #batch.append(SpawnActor(blueprint, transform)
                #    .then(SetVehicleLightState(FutureActor, light_state)))

                print(blueprint)
                spawn_zone = random.choice(ZONES)
                spawn_coordinates = random.choice(spawn_zone)
                spawn_point = carla.Transform(carla.Location(spawn_coordinates[0], spawn_coordinates[1], spawn_coordinates[2]), carla.Rotation(yaw=0))
                print("Spawn Zone:" , zone_name(spawn_zone), "\n", "Spawn Location: ", spawn_coordinates)
                vehicle = world.spawn_actor(blueprint, spawn_point)

                prev_location = spawn_coordinates
                actor_agent = BasicAgent(vehicle)
                next_waypoint = next_location(actor_agent, prev_location)
                prev_location = next_waypoint

                while True:
    
                    vehicle.apply_control(actor_agent.run_step())
                    world.wait_for_tick()
        
                    curr_location = vehicle.get_location()
                    curr_distance = sqrt(   (next_waypoint[0] - curr_location.x)**2 + (next_waypoint[1] - curr_location.y)**2      )
                    print("Current Distance: ", curr_distance)

                    if (curr_distance < 15):

                        prev_location = next_waypoint
                        next_waypoint = next_location(actor_agent, prev_location)
 
                        actor_list.append(vehicle)
                        time.sleep(5)    

            finally:
                for actor in actor_list:
                    actor.destroy()
                    print("All cleaned up!")

        # -------------
        # Spawn Walkers
        # -------------
        # some settings
        percentagePedestriansRunning = 0.0      # how many pedestrians will run
        percentagePedestriansCrossing = 0.0     # how many pedestrians will walk through the road
        # 1. take all the random locations to spawn
        spawn_points = []
        for i in range(args.number_of_walkers):
            spawn_point = carla.Transform()
            loc = world.get_random_location_from_navigation()
            if (loc != None):
                spawn_point.location = loc
                spawn_points.append(spawn_point)
        # 2. we spawn the walker object
        batch = []
        walker_speed = []
        for spawn_point in spawn_points:
            walker_bp = random.choice(blueprintsWalkers)
            # set as not invincible
            if walker_bp.has_attribute('is_invincible'):
                walker_bp.set_attribute('is_invincible', 'false')
            # set the max speed
            if walker_bp.has_attribute('speed'):
                if (random.random() > percentagePedestriansRunning):
                    # walking
                    walker_speed.append(walker_bp.get_attribute('speed').recommended_values[1])
                else:
                    # running
                    walker_speed.append(walker_bp.get_attribute('speed').recommended_values[2])
            else:
                print("Walker has no speed")
                walker_speed.append(0.0)
            batch.append(SpawnActor(walker_bp, spawn_point))
        results = client.apply_batch_sync(batch, True)
        walker_speed2 = []
        for i in range(len(results)):
            if results[i].error:
                logging.error(results[i].error)
            else:
                walkers_list.append({"id": results[i].actor_id})
                walker_speed2.append(walker_speed[i])
        walker_speed = walker_speed2
        # 3. we spawn the walker controller
        batch = []
        walker_controller_bp = world.get_blueprint_library().find('controller.ai.walker')
        for i in range(len(walkers_list)):
            batch.append(SpawnActor(walker_controller_bp, carla.Transform(), walkers_list[i]["id"]))
        results = client.apply_batch_sync(batch, True)
        for i in range(len(results)):
            if results[i].error:
                logging.error(results[i].error)
            else:
                walkers_list[i]["con"] = results[i].actor_id
        # 4. we put altogether the walkers and controllers id to get the objects from their id
        for i in range(len(walkers_list)):
            all_id.append(walkers_list[i]["con"])
            all_id.append(walkers_list[i]["id"])
        all_actors = world.get_actors(all_id)

        # wait for a tick to ensure client receives the last transform of the walkers we have just created
        if not args.sync or not synchronous_master:
            world.wait_for_tick()
        else:
            world.tick()

        # 5. initialize each controller and set target to walk to (list is [controler, actor, controller, actor ...])
        # set how many pedestrians can cross the road
        world.set_pedestrians_cross_factor(percentagePedestriansCrossing)
        for i in range(0, len(all_id), 2):
            # start walker
            all_actors[i].start()
            # set walk to random point
            all_actors[i].go_to_location(world.get_random_location_from_navigation())
            # max speed
            all_actors[i].set_max_speed(float(walker_speed[int(i/2)]))

        print('spawned %d vehicles and %d walkers, press Ctrl+C to exit.' % (len(vehicles_list), len(walkers_list)))

        # example of how to use parameters
        traffic_manager.global_percentage_speed_difference(30.0)

        while True:
            if args.sync and synchronous_master:
                world.tick()
            else:
                world.wait_for_tick()

    finally:

        if args.sync and synchronous_master:
            settings = world.get_settings()
            settings.synchronous_mode = False
            settings.fixed_delta_seconds = None
            world.apply_settings(settings)

        print('\ndestroying %d vehicles' % len(vehicles_list))
        client.apply_batch([carla.command.DestroyActor(x) for x in vehicles_list])

        # stop walker controllers (list is [controller, actor, controller, actor ...])
        for i in range(0, len(all_id), 2):
            all_actors[i].stop()

        print('\ndestroying %d walkers' % len(walkers_list))
        client.apply_batch([carla.command.DestroyActor(x) for x in all_id])

        time.sleep(0.5)

if __name__ == '__main__':

    try:
        main()
    except KeyboardInterrupt:
        pass
    finally:
        print('\ndone.')
