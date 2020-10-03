__copyright__ = "Copyright 2019, bThere.ai"

# This file is a utility wrapper for logging. It helps with log levels and logging to console plus file

import datetime
import sys
import logging
import logging.handlers
import os

LOG_DIR = '/var/log/bthere/'
LOG_FILENAME = os.path.join(LOG_DIR, 'bthere.log')
LOG_FILE_SIZE = 200000
logger = None

LOG = True
LOG_LEVELS = {"v": 0, "i": 1, "d": 2, "w": 3, "e": 4}
LOG_LEVEL = "i"  # Default log level

# Setup up LOGLEVEL before using it
raw_args = sys.argv[1:]
for arg in raw_args:
    arg_parts = arg.split('=')
    if arg_parts[0] == 'log_level' and len(arg_parts) > 1 and arg_parts[1] in LOG_LEVELS:
        LOG_LEVEL = arg_parts[1]


def log(level, log_message):
    global LOG
    global LOG_LEVEL
    global LOG_LEVELS
    # Log to stdout
    if LOG and LOG_LEVELS[level] >= LOG_LEVELS[LOG_LEVEL]:
        print(format_log(level, log_message))
        sys.stdout.flush()
    # Log to file
    if LOG and ((LOG_LEVELS[level] >= LOG_LEVELS["d"]) or (LOG_LEVELS[level] >= LOG_LEVELS[LOG_LEVEL])):
        logger.debug(format_log(level, log_message))


def format_log(level, log_message):
    now = datetime.datetime.utcnow()
    return str(now) + " " + level + ": " + str(log_message)


def v(log_message):
    log("v", log_message)


def i(log_message):
    log("i", log_message)


def d(log_message):
    log("d", log_message)


def w(log_message):
    log("w", log_message)


def e(log_message):
    log("e", log_message)


def init_logging():
    global logger
    logger = logging.getLogger("bthere_servo_node")
    logger.setLevel(logging.DEBUG)
    handler = logging.handlers.RotatingFileHandler(
        LOG_FILENAME, maxBytes=LOG_FILE_SIZE, backupCount=5)
    logger.addHandler(handler)


init_logging()
