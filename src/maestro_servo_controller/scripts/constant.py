__copyright__ = "Copyright 2019, bThere.ai"

# This file defines a couple base constants

import rospy
import bthere_log
import xml.etree.ElementTree as ET
import os

# General system constants
dir_path = os.path.dirname(os.path.realpath(__file__))
DEFAULT_CONFIG_FILE = dir_path + '/bthere.cfg'

VERSION = 'unknown'
NAME = 'maestro_servo_controller'

try:
    bthere_log.i("Looking for " + dir_path + "/../package.xml")
    tree = ET.parse(dir_path + "/../package.xml")
    bthere_log.i("Parsed package.xml")
    root = tree.getroot()
    version_node = root.find("version")
    if version_node is not None:
        VERSION = version_node.text
        bthere_log.i("Found version in package.xml")
    name_node = root.find("name")
    if name_node is not None:
        NAME = name_node.text
        bthere_log.i("Found name in package.xml")
except:
    bthere_log.w("Can't read package.xml")
bthere_log.i("VERSION: " + VERSION)
bthere_log.i("NAME: " + NAME)
