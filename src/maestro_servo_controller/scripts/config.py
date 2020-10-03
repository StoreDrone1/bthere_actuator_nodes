__copyright__ = "Copyright 2019, bThere.ai"

# This file loads and merges config from command line + base bthere.cfg + platform specific log file


from constant import DEFAULT_CONFIG_FILE, NAME, VERSION
import bthere_log
import sys
import json
import os
from jsmin import jsmin

args = {}

raw_args = sys.argv[1:]

if len(raw_args) == 1:
    if raw_args[0] == '--version':
        print(NAME + ' ' + VERSION)
        sys.exit()
    if raw_args[0] == '--help':
        print(NAME + ' ' + VERSION)
        print("Suggested args:")
        print("\tplatform=[linux|mac|RPi]")
        print(
            "\tmock_servos=[True|False]\tForces use of mock for servo control")
        print("\tconfig_file={name of a json file with config keys and values}\tNote that in json true and false are"
              " not capitalized")
        print("Example:")
        print("\bthere_servos mock_servos=False platform=linux "
              "config_file=bthere.cfg run_direct=True")
        sys.exit()


def get(key):
    if key in args:
        return args[key]
    else:
        return None


def get_config_or_default(key, default):
    if key in args:
        bthere_log.i("Found desired config value for " +
                     key + " so returning " + str(args[key]))
        return args[key]
    else:
        bthere_log.i("Didn't find desired config value for " +
                     key + " so returning default: " + str(default))
        return default


def json_stripped(filename):
    # Read original config file and strip out comments
    f = open(filename)
    json_str = jsmin(f.read())
    f.close()

    # Save the json to a temporary file.  This is done because we have mixtures of single & double quotes in the
    # json_str and json.load() baulks on it.
    tmp_filename = os.path.join('/tmp/', os.path.split(filename)[1] + "_tmp")
    f = open(tmp_filename, "w")
    f.write(json_str)
    f.close()

    # Read the stripped config file and delete it when done
    f = open(tmp_filename)
    json_values = json.load(f)
    f.close()
    os.remove(tmp_filename)
    return json_values


for arg in raw_args:
    arg_parts = arg.split('=')
    if len(arg_parts) > 1:
        if arg_parts[1] == 'True':
            args[arg_parts[0]] = True
        elif arg_parts[1] == 'False':
            args[arg_parts[0]] = False
        else:
            args[arg_parts[0]] = arg_parts[1]
        bthere_log.i("From command line setting " +
                     arg_parts[0] + " to " + arg_parts[1])
    else:
        args[arg_parts[0]] = True
        bthere_log.i("From command line setting " + arg_parts[0] + " to True")

bthere_log.i("Command line args: " + str(args))

config_file = get_config_or_default('config_file', DEFAULT_CONFIG_FILE)
if config_file is not None:
    bthere_log.i("Looking for config file: " + config_file)
    try:
        config_values = json_stripped(config_file)
        bthere_log.i("Config file values: " + str(config_values))
        args.update(config_values)
    except IOError:
        bthere_log.w("Could not find configuration file: " + config_file)
    config_path, config_filename = os.path.split(config_file)
    platform_config_filename = os.path.join(
        config_path, get('platform') + "_" + config_filename)
    bthere_log.i("Looking for config file: " + platform_config_filename)
    try:
        config_values = json_stripped(platform_config_filename)
        bthere_log.i("Platform config file values: " + str(config_values))
        args.update(config_values)
    except IOError:
        bthere_log.w("Could not find configuration file: " +
                     platform_config_filename)
    bthere_log.i("Merged config. Final set: " + str(args))
