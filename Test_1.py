
import heapq
class RobotEnvironment:
    def __init__(self, width, height, obstacles, charging_station):
        self.width = width
        self.height = height
        self.obstacles = set(obstacles)  # Set of (x, y) tuples
        self.charging_station = charging_station  # (x, y) tuple
        self.grid = [[" " for _ in range(width)] for _ in range(height)]
        for obs in obstacles:
            self.grid[obs[1]][obs[0]] = "O"
        self.grid[charging_station[1]][charging_station[0]] = "C"

    def print_grid(self):
        for row in self.grid:
            print(" ".join(row))
        print()

# Example instantiation
environment = RobotEnvironment(10, 10, [(2, 2), (3, 3), (4, 4)], (9, 9))
environment.print_grid()

def movement_cost(current_position, next_position, terrain_type):
    # Example: simple cost function based on terrain type
    if terrain_type == "carpet":
        return 1.5  # Higher cost on carpet
    return 1  # Default cost

def heuristic(current_position, target_position):
    # Manhattan distance as a simple heuristic
    return abs(current_position[0] - target_position[0]) + abs(current_position[1] - target_position[1])

# More sophisticated heuristic could factor in obstacles, potentially using a precomputed pathfinding grid

def a_star_search(start, goal, environment):
    open_set = []
    heapq.heappush(open_set, (0, start))
    came_from = {}
    g_score = {start: 0}
    f_score = {start: heuristic(start, goal)}

    while open_set:
        current = heapq.heappop(open_set)[1]

        if current == goal:
            path = []
            while current in came_from:
                path.append(current)
                current = came_from[current]
            return path[::-1]  # Return reversed path

        for neighbor in environment.get_neighbors(current):
            tentative_g_score = g_score[current] + movement_cost(current, neighbor, "carpet")  # Example usage

            if neighbor not in g_score or tentative_g_score < g_score[neighbor]:
                came_from[neighbor] = current
                g_score[neighbor] = tentative_g_score
                f_score[neighbor] = tentative_g_score + heuristic(neighbor, goal)
                if neighbor not in [i[1] for i in open_set]:
                    heapq.heappush(open_set, (f_score[neighbor], neighbor))

    return None

# Add get_neighbors to RobotEnvironment for this to work
def visualize_path(self, path):
    for y in range(self.height):
        for x in range(self.width):
            if (x, y) == self.charging_station:
                print("C", end=" ")
            elif (x, y) in self.obstacles:
                print("O", end=" ")
            elif (x, y) in path:
                print(".", end=" ")
            else:
                print(" ", end=" ")
        print()
