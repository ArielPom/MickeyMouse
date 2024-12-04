import socket
import json
import gen_commands_wall
import maze_solver
import maze_solver_less_changes
import numpy as np
import math

N = 5 # Define the size of the grid
rectangle_length_mm = 200  # Define the length of the rectangles in mm

# UDP setup
udp_ip = ""  # Empty string to listen on all available interfaces
udp_port = 5555       # Replace with the port to listen on
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind((udp_ip, udp_port))

print(f"Listening for UDP packets on {udp_ip}:{udp_port}")

def read_udp_maze():

    while True:
        # Receive edges via UDP
        data, addr = sock.recvfrom(1024)  # Buffer size is 1024 bytes
        edges = json.loads(data.decode())
        print("Received edges:", edges)

        # Receive start and end coordinates via UDP
        data, addr = sock.recvfrom(1024)  # Buffer size is 1024 bytes
        start_end_coords = json.loads(data.decode())
        start_coords, end_coords = start_end_coords
        print("Received start and end coordinates:", start_coords, end_coords)

        # Convert start and end coordinates to tuples
        start_coords = (start_coords[0], start_coords[1])
        end_coords = (end_coords[0], end_coords[1])

        # Convert edges to tuples
        edges = [(tuple(edge[0]), tuple(edge[1])) for edge in edges]
        
        # Perform A* search
        # path = maze_solver.a_star_search(start_coords, end_coords, sorted_edges)
        path = maze_solver_less_changes.a_star_search(start_coords, end_coords, edges)
        print("Shortest path:", path)


        commands = gen_commands_wall.generate_commands(path, rectangle_length_mm, edges, N)
        # print the commands with new line between each command
        print("Commands:")
        for command in commands:
            print(command)

        print("type of commands: ", type(commands))
        return commands
        
if __name__ == "__main__":
    trajectory = read_udp_maze()
    print(trajectory)