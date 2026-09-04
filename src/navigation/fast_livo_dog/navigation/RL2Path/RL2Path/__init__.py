# Copyright (c) 2022-2025, The Isaac Lab Project Developers (https://github.com/isaac-sim/IsaacLab/blob/main/CONTRIBUTORS.md).
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""
Python module serving as a project/extension template.
"""

# Register Gym environments.
try:
    from .tasks import *
except Exception as e:
    print(f"isaac is not launched, skipping Gym registration: {e}")

# Register UI extensions.
# from .ui_extension_example import *
