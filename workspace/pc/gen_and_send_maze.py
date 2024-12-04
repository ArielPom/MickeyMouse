import matplotlib.pyplot as plt
import numpy as np
from matplotlib.widgets import Button
import gen_commands
import maze_solver
import socket
import json

# Define the size of the grid
N = 5
border_threshold = 0.1  # Define a threshold distance to consider a click close to the border
middle_threshold = 0.2  # Define a threshold to consider a click in the middle area
chosen_border_width = 4  # Define the width of the border lines
rectangle_length_mm = 200  # Define the length of the rectangles in mm

# UDP setup
PI_IP_ADDRESS = "127.0.0.1"
PI_IP_ADDRESS = "192.168.1.136"
PI_PORT = 5555
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

# Create a figure and axis object
fig, ax = plt.subplots()
plt.subplots_adjust(bottom=0.2)

# Set the axis limits to match the size of the grid
ax.set_xlim(0, N)
ax.set_ylim(0, N)

# Set the axis ticks to match the size of the grid
ax.set_xticks(np.arange(N+1))
ax.set_yticks(np.arange(N+1))

# Create a list to store the rectangles' border lines and the rectangles themselves
border_lines = []
rectangles = []
line_to_rect_map = {}

# Counter for numbering the rectangles
rect_counter = 0

# init start and end rect of maze
start_rect = None
end_rect = None


def draw_grid():
    # Draw white rectangles and their borders to represent the grid cells
    for i in range(N):
        for j in range(N):
            # Create a rectangle
            rect = plt.Rectangle((j, i), 1, 1, edgecolor='none', facecolor='white', picker=True)
            ax.add_patch(rect)
            rectangles.append(rect)

            # Add coordinates text to each rectangle
            coord_text = f"{j},{i}"
            ax.text(j + 0.5, i + 0.5, coord_text, color='black', ha='center', va='center')

            # Create lines for each border
            left_line = plt.Line2D([j, j], [i, i+1], color='black', linewidth=1, picker=True)
            right_line = plt.Line2D([j+1, j+1], [i, i+1], color='black', linewidth=1, picker=True)
            bottom_line = plt.Line2D([j, j+1], [i, i], color='black', linewidth=1, picker=True)
            top_line = plt.Line2D([j, j+1], [i+1, i+1], color='black', linewidth=1, picker=True)
            
            # Add lines to the axis
            ax.add_line(left_line)
            ax.add_line(right_line)
            ax.add_line(bottom_line)
            ax.add_line(top_line)
            
            # Store the lines in the list
            border_lines.extend([left_line, right_line, bottom_line, top_line])
            
            # Map lines to their corresponding rectangles
            line_to_rect_map[left_line] = ((j, i), (j-1, i))
            line_to_rect_map[right_line] = ((j, i), (j+1, i))
            line_to_rect_map[bottom_line] = ((j, i), (j, i-1))
            line_to_rect_map[top_line] = ((j, i), (j, i+1))

    # Set outer border lines to red initially
    outer_border_lines = [
        plt.Line2D([0, 0], [0, N], color='red', linewidth=chosen_border_width),           # Left outer border
        plt.Line2D([N, N], [0, N], color='red', linewidth=chosen_border_width),           # Right outer border
        plt.Line2D([0, N], [0, 0], color='red', linewidth=chosen_border_width),           # Bottom outer border
        plt.Line2D([0, N], [N, N], color='red', linewidth=chosen_border_width)            # Top outer border
    ]

    # Add outer border lines to the axis
    for line in outer_border_lines:
        ax.add_line(line)


# Function to handle mouse clicks
def onpick(event):
    global rect_counter
    global start_rect
    global end_rect
    # Check if the event is within the axes
    if event.inaxes == ax:
        for line in border_lines:
            # Check if the click was near the line
            if line.contains(event)[0]:
                # Change the clicked line color to red and make it bold
                line.set_color('red')
                line.set_linewidth(chosen_border_width)
                fig.canvas.draw()

        for rect in rectangles:
            x, y = rect.get_xy()
            width, height = rect.get_width(), rect.get_height()
            middle_x, middle_y = x + width / 2, y + height / 2
            threshold_x = width * middle_threshold
            threshold_y = height * middle_threshold
            
            if (middle_x - threshold_x < event.xdata < middle_x + threshold_x and
                middle_y - threshold_y < event.ydata < middle_y + threshold_y):
                # Color the rectangle yellow and assign a number
                if rect_counter == 0:
                    start_rect = rect
                    rect.set_facecolor('yellow')
                else:
                    end_rect = rect
                    rect.set_facecolor('green')
                rect_counter += 1
                fig.canvas.draw()
                return  # Exit after updating the rectangle


def send_maze_udp(edges, start_coords, end_coords):
    # Send edges via UDP
    edges_json = json.dumps(edges)
    sock.sendto(edges_json.encode(), (PI_IP_ADDRESS, PI_PORT))
    print("Edges sent via UDP to ip:", PI_IP_ADDRESS, "port:", PI_PORT)

    # send start and end coordinates via UDP
    start_end_coords = json.dumps([start_coords, end_coords])
    sock.sendto(start_end_coords.encode(), (PI_IP_ADDRESS, PI_PORT))
    print("Start and end coordinates sent via UDP:", start_coords, end_coords)
    return

# Function to check for red lines and store edges
def solve_maze(event):
    edges = set()
    for line in border_lines:
        if line.get_color() != 'red':
            rect1, rect2 = line_to_rect_map[line]
            if 0 <= rect1[0] < N and 0 <= rect1[1] < N and 0 <= rect2[0] < N and 0 <= rect2[1] < N:
                new_edge = tuple(sorted((rect1, rect2)))
                edges.add(new_edge)
    sorted_edges = sorted(edges, key=lambda edge: (edge[0], edge[1]))
    print("Edges without red lines:", sorted_edges)
    print("Start rect:", start_rect)
    print("End rect:", end_rect)
    print()
    
    start_coords = (int(start_rect.get_x()), int(start_rect.get_y()))
    end_coords = (int(end_rect.get_x()), int(end_rect.get_y()))

    # Send the maze to the PI
    send_maze_udp(sorted_edges, start_coords, end_coords)

    # Perform A* search
    # path = maze_solver.a_star_search(start_coords, end_coords, sorted_edges)
    path = maze_solver.a_star_search(start_coords, end_coords, sorted_edges)
    print("Shortest path:", path)

    commands = gen_commands.generate_commands(path, rectangle_length_mm)
    # print the commands with new line between each command
    print("Commands:")
    for command in commands:
        print(command)

    # find all rectangles in the path and change thier color to grey (except for start and end rect)
    for xy in path:
        for rect in rectangles:
            x, y = rect.get_xy()
            if (x, y) == xy and rect != start_rect and rect != end_rect:
                rect.set_facecolor('grey')
                fig.canvas.draw()
                # wait 0.2 seconds
                plt.pause(0.4)
    

    return sorted_edges, path



if __name__ == "__main__":

    # Draw the maze
    draw_grid()

    # Connect the onpick function to the button_press_event
    fig.canvas.mpl_connect('button_press_event', onpick)

    # Add a button to trigger the edge check
    ax_check = plt.axes([0.7, 0.05, 0.2, 0.075])
    btn_check = Button(ax_check, 'Solve')
    btn_check.on_clicked(solve_maze)
    
    try:
        plt.show()
    except KeyboardInterrupt:
        print("Exiting...")
        exit(0)