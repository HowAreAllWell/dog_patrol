import os
import pickle
import argparse

# from isaaclab.app import AppLauncher

# # add argparse arguments
# parser = argparse.ArgumentParser(
#     description="This script demonstrates adding a custom robot to an Isaac Lab environment."
# )
# parser.add_argument("--num_envs", type=int, default=1, help="Number of environments to spawn.")
# # append AppLauncher cli args
# AppLauncher.add_app_launcher_args(parser)
# # parse the arguments
# args_cli = parser.parse_args()

# # launch omniverse app
# app_launcher = AppLauncher(args_cli)
# simulation_app = app_launcher.app

from RL2Path.nav.grid_2d import Grid, GridCfg
from RL2Path.utils.generate_obj import generate_obj

cfg = GridCfg(
    x_min = -5.0,
    x_max = 5.0,
    y_min = -5.0,
    y_max = 5.0,
    grid_size = 0.01,
    grid_inflated_distance = 1.0,  # Default inflation distance
    wall_thickness = 0.2,  # Wall thickness
    num_large_rect = 2,
    num_large_circ = 2,
    num_small_rect = 8,
    num_small_circ = 8,
    large_rect_size = (0.5, 1.2),  # Width and height range for large rectangles
    large_circ_radius = (0.55, 0.6),  # Radius range for large circles
    small_rect_size = (0.1, 0.5), # Width and height range for small rectangles
    small_circ_radius = (0.05, 0.1),  # Radius range for small circles
    min_distance = 0.5,
)

grid = Grid(cfg)

grid.generate_obstacles()

scene_name = "scene_4"

print(len(grid.obstacles))

if not os.path.exists(f"./outputs/{scene_name}"):
    os.makedirs(f"./outputs/{scene_name}")
    
generate_obj(grid.obstacles, ouput_dir=f"./outputs/{scene_name}")

with open(f"./outputs/{scene_name}/grid.pkl", "wb") as f:
    pickle.dump(grid, f)


    

