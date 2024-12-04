import numpy as np

def find_closest_wall(edges, current_position, direction, rectangle_length_mm, grid_size):
    """Find the closest wall in the movement direction from the current position."""
    x, y = current_position
    distance = 0
    while True:
        distance += 1
        # print(distance)
        # Determine the next position based on the direction of movement
        x, y = current_position
        if direction == 'x+':
            next_pos = (x + 1, y)
        elif direction == 'x-':
            next_pos = (x - 1, y)
        elif direction == 'y+':
            next_pos = (x, y + 1)
        elif direction == 'y-':
            next_pos = (x, y - 1)
        # print(current_position, next_pos, direction)
        # Check if the next position is out of bounds (grid boundary)
        if next_pos[0] < 0 or next_pos[0] >= grid_size or next_pos[1] < 0 or next_pos[1] >= grid_size:
            break  # Reached grid boundary which counts as a wall

        # Check if there's no edge between the current and next position (indicating a wall)
        if tuple(sorted((current_position, next_pos))) not in edges:
            break  # Wall found due to missing edge

        # Move to the next position and continue
        current_position = next_pos
    return np.abs(distance * rectangle_length_mm)  # Return the distance to the nearest wall in mm


def generate_commands(path, rectangle_length_mm, edges, grid_size):
    current_theta = 0
    current_x = 0
    current_y = 0
    commands = []
    prev_dir = 'p'
    for i in range(1, len(path)):
        x_prev, y_prev = path[i-1]
        x_curr, y_curr = path[i]
        x_diff = (x_curr - x_prev) * rectangle_length_mm
        y_diff = (y_curr - y_prev) * rectangle_length_mm

        if x_diff != 0:
            # Determine direction based on x_diff
            direction = 'x+' if x_diff > 0 else 'x-'
            dst_theta = (0 if x_diff > 0 else np.pi)

            # Rotate if needed
            if current_theta != dst_theta:
                current_theta = dst_theta
                command = {"x": current_x, "y": current_y, "theta": current_theta, "operation": 1, "wall": 0}
                commands.append(command)

            # Move forward and calculate the closest wall in the x-direction
            current_x += x_diff
            if prev_dir != direction:
                closest_wall = find_closest_wall(edges, (x_prev, y_prev), direction, rectangle_length_mm, grid_size)
                prev_dir = direction
            command = {"x": current_x, "y": current_y, "theta": current_theta, "operation": 0, "wall": closest_wall}
            commands.append(command)

        if y_diff != 0:
            # Determine direction based on y_diff
            direction = 'y+' if y_diff > 0 else 'y-'
            dst_theta = (np.pi/2 if y_diff > 0 else 3*np.pi/2)

            # Rotate if needed
            if current_theta != dst_theta:
                current_theta = dst_theta
                command = {"x": current_x, "y": current_y, "theta": current_theta, "operation": 1, "wall": 0}
                commands.append(command)

            # Move forward and calculate the closest wall in the y-direction
            current_y += y_diff
            if prev_dir != direction:
                closest_wall = find_closest_wall(edges, (x_prev, y_prev), direction, rectangle_length_mm, grid_size)
                prev_dir = direction
            command = {"x": current_x, "y": current_y, "theta": current_theta, "operation": 0, "wall": closest_wall}
            commands.append(command)

    # Merge consecutive forward commands
    merged_commands = []
    for command in commands:
        if len(merged_commands) == 0:
            merged_commands.append(command)
        else:
            if command["operation"] == 0 and merged_commands[-1]["operation"] == 0:
                merged_commands[-1]["x"] = command["x"]
                merged_commands[-1]["y"] = command["y"]
                merged_commands[-1]["wall"] = command["wall"]
            else:
                merged_commands.append(command)

    return merged_commands
