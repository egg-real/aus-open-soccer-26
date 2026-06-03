import json

# globals

config_file = "config.json"
_globals = {}

def load_config():
    global _globals
    with open(config_file, "r") as f:
        _globals = json.load(f)

def save_config():
    with open(config_file, "w") as f:
        json.dump(_globals, f, indent=4)

def get_value(key, default=None):
    return _globals.get(key, default)

def set_value(key, value):
    _globals[key] = value
    return

load_config()