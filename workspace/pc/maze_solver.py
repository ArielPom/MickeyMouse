import heapq

TURNING_PENALTY = 2

def calc_dist(a, b):
    """Calculate the Manhattan distance between points a and b."""
    return abs(a[0] - b[0]) + abs(a[1] - b[1])

def get_direction(a, b):
    """Get the direction of movement from point a to point b."""
    if a[0] == b[0]:
        return 'horizontal'
    elif a[1] == b[1]:
        return 'vertical'
    else:
        return 'other'  # This should not happen in a grid

def a_star_search(start, goal, edges):
    """Perform A* search from start to goal on the given edges with fewer direction changes."""
    # Create adjacency list from edges
    graph = {}
    for edge in edges:
        (a, b) = edge
        if a not in graph:
            graph[a] = []
        if b not in graph:
            graph[b] = []
        graph[a].append(b)
        graph[b].append(a)

    # Priority queue for the open set
    open_set = []
    heapq.heappush(open_set, (0, start, None))  # (f_score, current_node, previous_direction)

    # Maps to keep track of the cost of reaching each node and the path taken
    g_score = {start: 0}
    f_score = {start: calc_dist(start, goal)}
    came_from = {}

    while open_set:
        # Get the node in open_set having the lowest f_score value
        _, current, current_direction = heapq.heappop(open_set)

        # If we've reached the goal, reconstruct the path
        if current == goal:
            path = []
            while current in came_from:
                path.append(current)
                current = came_from[current]
            path.append(start)
            path.reverse()
            return path

        # For each neighbor of the current node
        for neighbor in graph.get(current, []):
            tentative_g_score = g_score[current] + 1
            direction = get_direction(current, neighbor)
            if current_direction and direction != current_direction:
                tentative_g_score += TURNING_PENALTY  # Add penalty for direction change

            if tentative_g_score < g_score.get(neighbor, float('inf')):
                # This path to neighbor is better than any previous one. Record it!
                came_from[neighbor] = current
                g_score[neighbor] = tentative_g_score
                f_score[neighbor] = tentative_g_score + calc_dist(neighbor, goal)
                heapq.heappush(open_set, (f_score[neighbor], neighbor, direction))

    # If we get here, there is no path
    return None