#!/usr/bin/env python3

import matplotlib.pyplot as plt
import pandas as pd
import numpy as np
import sys

log_path = sys.argv[1]
log_data = pd.read_csv(log_path)

print(log_data)