import json
import math

from bumperbot_digital_twin.nav_preview import json_safe


def test_json_safe_replaces_nan_and_inf():
    data = {"a": math.nan, "b": [1.0, math.inf, {"c": -math.inf}], "d": "x", "e": 2}
    text = json.dumps(json_safe(data), allow_nan=False)
    assert json.loads(text) == {"a": None, "b": [1.0, None, {"c": None}], "d": "x", "e": 2}
