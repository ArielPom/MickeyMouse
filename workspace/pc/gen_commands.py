# Description: Generate commands for the robot to follow the path
import numpy as np

def generate_commands(path, rectangle_length_mm):
    # loop through path from second element to the last element
    # and calc {"x": x_dist, "y": y_dist, "theta": theta, "operation": 0/1}
    # where 0 is forward and 1 is rotate
    current_theta = 0
    current_x = 0
    current_y = 0
    commands = []
    for i in range(1, len(path)):
        x_prev, y_prev = path[i-1]
        x_curr, y_curr = path[i]
        x_diff, y_diff = (x_curr - x_prev) * rectangle_length_mm, (y_curr - y_prev) * rectangle_length_mm

        if x_diff != 0:
            dst_theta = (0 if x_diff > 0 else np.pi)

            # rotate if neededs
            if current_theta != dst_theta:
                current_theta = dst_theta
                command = {"x": current_x, "y": current_y, "theta": current_theta, "operation": 1}
                commands.append(command)
            # move forward
            current_x += x_diff
            command = {"x": current_x, "y": current_y, "theta": current_theta, "operation": 0}
            commands.append(command)

        if y_diff != 0:
            dst_theta = (np.pi/2 if y_diff > 0 else 3*np.pi/2)

            # rotate if needed
            if current_theta != dst_theta:
                current_theta = dst_theta
                command = {"x": current_x, "y": current_y, "theta": current_theta, "operation": 1}
                commands.append(command)
            # move forward
            current_y += y_diff
            command = {"x": current_x, "y": current_y, "theta": current_theta, "operation": 0}
            commands.append(command)
    
    # merge consecutive forward commands
    merged_commands = []
    for command in commands:
        if len(merged_commands) == 0:
            merged_commands.append(command)
        else:
            if command["operation"] == 0 and merged_commands[-1]["operation"] == 0:
                merged_commands[-1]["x"] = command["x"]
                merged_commands[-1]["y"] = command["y"]
            else:
                merged_commands.append(command)
    return merged_commands