
def user_input(is_robot_rigid):
    print(">>> Consider bottom left corner as (0,0).. Also do consider the clearance from boundary while entering the positions.")
    if is_robot_rigid:
        print(">>> Enter the Initial and Target positions, Initial Orientation, Radius and Clearance, and Step-size required for the robot")
    else:
        print(">>> Enter the Initial and Target positions, Initial Orientation and Step-size for the robot")
    print(">> Example: if position is (x=20, y=20), then your input should be '20 20'")
    
    init_pos_str = input("Initial position (x y): ")
    orientation_str = input("Initial Orientation (in degrees): ")
    target_pos_str = input("Target position (x y): ")
    orientation_goal = input("Goal Orientation (in degrees): ")
    step_size_str = input("Step size: ")
    
    radius_str = '0'
    clearance_str = '0'

    # Take radius and clearance input for rigid robot.
    if is_robot_rigid:
        radius_str = input("Robot Radius: ")
        clearance_str = input("Clearance: ")
    
    is_input_valid = True
    try:
        init_pos = [int(coord) for coord in init_pos_str.split()]
        target_pos = [int(coord) for coord in target_pos_str.split()]
        orientation = float(orientation_str)
        step_size = float(step_size_str)
        goal_direction = float(orientation_goal)

        robot_radius = int(radius_str)
        clearance_req = int(clearance_str)

        if len(init_pos) != 2 or len(target_pos) != 2:
            raise Exception("Invalid input")
        if not (1 <= step_size <= 50):
            raise Exception("Invalid input")

    except Exception as e:
        is_input_valid = False
        print("Error: ", e)
        print('Invalid input. Try again!')
    
    return is_input_valid, init_pos, target_pos, orientation, goal_direction, robot_radius, clearance_req, step_size