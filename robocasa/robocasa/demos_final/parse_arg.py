import json
import numpy as np
import sys
import os

data=sys.argv[1]
total_time = ""
if len(sys.argv) > 2:
    total_time="_"+sys.argv[2]

with open(f"robocasa/demos/goal_state_file{total_time}", "w") as f:
    print(data, file=f)
