import math
import numpy as np
from input_receiver import user_input
from pathexplorer import PathExplorer
from configurationspace import ConfigurationSpace
from heuristics import EUCL_HEURISTIC, MANHTN_HEURISTIC


if __name__ == "__main__":

    is_input_valid, init_pos, target_pos, orientation, goal_direction, robot_radius, obstacle_clearance, step_size = user_input(is_robot_rigid = True)

    if is_input_valid:
        init_pos = init_pos[1], init_pos[0]
        target_pos = target_pos[1], target_pos[0]

        c_space = ConfigurationSpace(height=250, width=400, radius_of_bot=robot_radius, clearance=obstacle_clearance)
        path_explorer = PathExplorer()
        path_explorer.find_path(init_pos, target_pos, orientation, step_size, c_space, EUCL_HEURISTIC)

        

     

