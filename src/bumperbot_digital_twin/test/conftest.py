import os
import re

HERE = os.path.dirname(__file__)


def bridge_columns(name="CSV_COLUMNS"):
    src = open(os.path.join(HERE, "..", "bumperbot_digital_twin", "twin_bridge.py")).read()
    return eval(re.search(name + r" = (\[.*?\])", src, re.S).group(1))
