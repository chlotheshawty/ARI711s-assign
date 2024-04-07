#!/usr/bin/env python
# coding: utf-8

# In[2]:


import heapq
import math

class Node:
    def __init__(self, position, parent=None, cost=0, heuristic=0):
        self.position = position
        self.parent = parent
        self.cost = cost
        self.heuristic = heuristic
        self.total_cost = cost + heuristic

    def __lt__(self, other):
        return self.total_cost < other.total_cost

def manhattan_distance(start, target):
    return abs(start[0] - target[0]) + abs(start[1] - target[1])

def calculate_heuristic(current_position, target_position, obstacles):
  
    distance = manhattan_distance(current_position, target_position)
    for obstacle in obstacles:
        distance += manhattan_distance(current_position, obstacle) * 0.5  # Penalize paths around obstacles
    return distance

def get_neighbors(current_node, obstacles, grid_width, grid_height):
    x, y = current_node.position
    neighbors = []
    for dx, dy in [(0, 1), (1, 0), (0, -1), (-1, 0)]:
        new_x, new_y = x + dx, y + dy
        if 0 <= new_x < grid_width and 0 <= new_y < grid_height and (new_x, new_y) not in obstacles:
            neighbors.append((new_x, new_y))
    return neighbors

def a_star_search(start, target, obstacles, grid_width, grid_height):
    open_list = []
    closed_set = set()
    start_node = Node(start, cost=0, heuristic=calculate_heuristic(start, target, obstacles))
    heapq.heappush(open_list, start_node)

    while open_list:
        current_node = heapq.heappop(open_list)

        if current_node.position == target:
            path = []
            while current_node:
                path.append(current_node.position)
                current_node = current_node.parent
            return path[::-1]

        closed_set.add(current_node.position)

        for neighbor_pos in get_neighbors(current_node, obstacles, grid_width, grid_height):
            if neighbor_pos in closed_set:
                continue

            neighbor_cost = current_node.cost + 1  # Assuming unit cost for each movement
            neighbor_node = Node(neighbor_pos, parent=current_node, cost=neighbor_cost,
                                 heuristic=calculate_heuristic(neighbor_pos, target, obstacles))
            if neighbor_node not in open_list:
                heapq.heappush(open_list, neighbor_node)

    return None

def test_environment(grid_width, grid_height, start, charging_station, obstacles):
    path = a_star_search(start, charging_station, obstacles, grid_width, grid_height)
    if path:
        print("Path found to charging station:", path)
        total_cost = len(path) - 1  # Total cost is the number of movements, excluding initial position
        print("Total cost:", total_cost)
    else:
        print("No path found to charging station")



print("Test Environment 1:")
grid_width = 10
grid_height = 10
start = (0, 0)
charging_station = (9, 9)
obstacles = [(2, 2), (3, 3), (4, 4), (5, 5)]
test_environment(grid_width, grid_height, start, charging_station, obstacles)

print("\nTest Environment 2:")
grid_width = 8
grid_height = 8
start = (1, 1)
charging_station = (6, 6)
obstacles = [(2, 2), (3, 2), (4, 2), (5, 2), (2, 3), (3, 3), (4, 3), (5, 3)]
test_environment(grid_width, grid_height, start, charging_station, obstacles)


print("\nAnalysis:")
print("When using the Manhattan distance heuristic only:")
print("- Longer paths are expected because the algorithm may not consider obstacles.")
print("- Path efficiency may be lower, especially when obstacles are present.")
print("\nWhen using the obstacle-aware heuristic:")
print("- Paths are likely to be more efficient as they consider obstacles.")
print("- Shorter paths are expected compared to the Manhattan distance heuristic.")


# In[11]:


import heapq
import math
import random
import time

class Node:
    def __init__(self, position, parent=None, cost=0, heuristic=0):
        self.position = position
        self.parent = parent
        self.cost = cost
        self.heuristic = heuristic
        self.total_cost = cost + heuristic

    def __lt__(self, other):
        return self.total_cost < other.total_cost

def manhattan_distance(start, target):
    return abs(start[0] - target[0]) + abs(start[1] - target[1])

def calculate_heuristic(current_position, target_position, obstacles):
    
    distance = manhattan_distance(current_position, target_position)
    for obstacle in obstacles:
        distance += manhattan_distance(current_position, obstacle) * 0.5 
    return distance

def get_neighbors(current_node, obstacles, grid_width, grid_height):
    x, y = current_node.position
    neighbors = []
    for dx, dy in [(0, 1), (1, 0), (0, -1), (-1, 0)]:
        new_x, new_y = x + dx, y + dy
        if 0 <= new_x < grid_width and 0 <= new_y < grid_height and (new_x, new_y) not in obstacles:
            neighbors.append((new_x, new_y))
    return neighbors

def a_star_search(start, target, obstacles, grid_width, grid_height):
    open_list = []
    closed_set = set()
    start_node = Node(start, cost=0, heuristic=calculate_heuristic(start, target, obstacles))
    heapq.heappush(open_list, start_node)

    while open_list:
        current_node = heapq.heappop(open_list)

        if current_node.position == target:
            path = []
            while current_node:
                path.append(current_node.position)
                current_node = current_node.parent
            return path[::-1]

        closed_set.add(current_node.position)

        for neighbor_pos in get_neighbors(current_node, obstacles, grid_width, grid_height):
            if neighbor_pos in closed_set:
                continue

            neighbor_cost = current_node.cost + 1 
            neighbor_node = Node(neighbor_pos, parent=current_node, cost=neighbor_cost,
                                 heuristic=calculate_heuristic(neighbor_pos, target, obstacles))
            if neighbor_node not in open_list:
                heapq.heappush(open_list, neighbor_node)

    return None

def visualize_environment(grid_width, grid_height, obstacles, path, cleaning_progress):
    for y in range(grid_height):
        for x in range(grid_width):
            if (x, y) in obstacles:
                print("X", end=" ")
            elif (x, y) in path:
                print("*", end=" ")
            elif (x, y) in cleaning_progress:
                print("C", end=" ")
            else:
                print("-", end=" ")
        print()

def simulate_cleaning(grid_width, grid_height, start, charging_station, obstacles):
    cleaning_progress = set()
    current_position = start
    while cleaning_progress != set(obstacles):
        path = a_star_search(current_position, charging_station, obstacles, grid_width, grid_height)
        if path:
            next_position = path[1]
            cleaning_progress.add(next_position)
            visualize_environment(grid_width, grid_height, obstacles, path, cleaning_progress)
            current_position = next_position
            time.sleep(1)  
            print("\033[H\033[J") 
        else:
            print("Cannot find a path to continue cleaning.")
            break


grid_width = 9
grid_height =9
start = (0, 0)
charging_station = (8 , 8)
obstacles = [(random.randint(0, grid_width - 1), random.randint(0, grid_height - 1)) for _ in range(15)]

simulate_cleaning(grid_width, grid_height, start, charging_station, obstacles)


# In[12]:


class Environment:
    def __init__(self, width, height):
        self.width = width
        self.height = height
        self.obstacles = set()  # Set to store coordinates of obstacles
    
    def add_obstacle(self, x, y):
        """
        Add obstacle at given coordinates.
        """
        if 0 <= x < self.width and 0 <= y < self.height:
            self.obstacles.add((x, y))
        else:
            print("Invalid obstacle coordinates")

    def is_obstacle(self, x, y):
        """
        Check if given coordinates have an obstacle.
        """
        return (x, y) in self.obstacles

class Robot:
    def __init__(self, environment, start_x, start_y):
        self.environment = environment
        self.x = start_x
        self.y = start_y
        self.direction = 'up'  # Robot's initial direction
    
    def move_forward(self):
        """
        Move the robot forward based on its current direction.
        """
        if self.direction == 'up' and self.y < self.environment.height - 1:
            self.y += 1
        elif self.direction == 'down' and self.y > 0:
            self.y -= 1
        elif self.direction == 'left' and self.x > 0:
            self.x -= 1
        elif self.direction == 'right' and self.x < self.environment.width - 1:
            self.x += 1

    def move_backward(self):
        """
        Move the robot backward based on its current direction.
        """
        if self.direction == 'up' and self.y > 0:
            self.y -= 1
        elif self.direction == 'down' and self.y < self.environment.height - 1:
            self.y += 1
        elif self.direction == 'left' and self.x < self.environment.width - 1:
            self.x += 1
        elif self.direction == 'right' and self.x > 0:
            self.x -= 1

    def turn_left(self):
        """
        Turn the robot left.
        """
        if self.direction == 'up':
            self.direction = 'left'
        elif self.direction == 'left':
            self.direction = 'down'
        elif self.direction == 'down':
            self.direction = 'right'
        elif self.direction == 'right':
            self.direction = 'up'

    def turn_right(self):
        """
        Turn the robot right.
        """
        if self.direction == 'up':
            self.direction = 'right'
        elif self.direction == 'right':
            self.direction = 'down'
        elif self.direction == 'down':
            self.direction = 'left'
        elif self.direction == 'left':
            self.direction = 'up'



# Create environment
env = Environment(9, 9)
env.add_obstacle(3, 4)
env.add_obstacle(7, 8)

# Create robot
robot = Robot(env, 1, 1)

# Perform actions
robot.move_forward()
robot.turn_right()
robot.move_forward()

print("Robot's current position:", robot.x, robot.y)
print("Robot's current direction:", robot.direction)
print("Is there an obstacle at (3, 4)?", env.is_obstacle(3, 4))


# In[13]:


class TSP:
    def __init__(self, distances):
        self.distances = distances
        self.num_places = len(distances)
    
    def calculate_total_distance(self, route):
        total_distance = 0
        for i in range(len(route) - 1):
            current_place = route[i]
            next_place = route[i + 1]
            total_distance += self.distances[current_place][next_place]
        # Add distance from last place back to starting place
        total_distance += self.distances[route[-1]][route[0]]
        return total_distance

    def hill_climbing(self, initial_route):
        current_route = initial_route
        current_distance = self.calculate_total_distance(current_route)

        while True:
            neighbors = self.get_neighbors(current_route)
            best_neighbor = min(neighbors, key=lambda x: self.calculate_total_distance(x))

            if self.calculate_total_distance(best_neighbor) >= current_distance:
                return current_route, current_distance
            else:
                current_route = best_neighbor
                current_distance = self.calculate_total_distance(current_route)

    def get_neighbors(self, route):
        neighbors = []
        for i in range(self.num_places):
            for j in range(i + 1, self.num_places):
                neighbor = route[:i] + [route[j]] + route[i+1:j] + [route[i]] + route[j+1:]
                neighbors.append(neighbor)
        return neighbors
distances = [
    [0, 7, 20, 15, 12],
    [10, 0, 6, 14, 18],
    [20, 6, 0, 15, 30],
    [15, 14, 25, 0, 2],
    [12, 18, 30, 2, 0]
]

tsp = TSP(distances)
initial_route = [0, 1, 2, 3, 4] 
optimal_route, optimal_distance = tsp.hill_climbing(initial_route)

print("Optimal Route:", optimal_route)
print("Optimal Distance:", optimal_distance, "km")


# In[14]:


import itertools
import math
import random

class TSPSolver:
    def __init__(self, cities):
        self.cities = cities
        self.num_cities = len(cities)

    def distance(self, city1, city2):
        return math.sqrt((city1[0] - city2[0])**2 + (city1[1] - city2[1])**2)

    def total_distance(self, tour):
        total_dist = 0
        for i in range(self.num_cities - 1):
            total_dist += self.distance(self.cities[tour[i]], self.cities[tour[i+1]])
        total_dist += self.distance(self.cities[tour[-1]], self.cities[tour[0]])  # Return to the starting point
        return total_dist

    def random_tour(self):
        return random.sample(range(self.num_cities), self.num_cities)

    def hill_climbing(self, max_iterations):
        current_tour = self.random_tour()
        current_dist = self.total_distance(current_tour)

        for _ in range(max_iterations):
            neighbor_tours = [self._generate_neighbor(current_tour) for _ in range(self.num_cities - 1)]
            neighbor_distances = [self.total_distance(tour) for tour in neighbor_tours]
            best_neighbor_dist = min(neighbor_distances)

            if best_neighbor_dist < current_dist:
                best_neighbor_idx = neighbor_distances.index(best_neighbor_dist)
                current_tour = neighbor_tours[best_neighbor_idx]
                current_dist = best_neighbor_dist
            else:
                break  # Reached a local minimum

        return current_tour, current_dist

    def _generate_neighbor(self, tour):
        i, j = sorted(random.sample(range(self.num_cities), 2))
        return tour[:i] + tour[j:j+1] + tour[i+1:j] + tour[i:i+1] + tour[j+1:]

# Test the TSP solver
cities = [(0, 0), (1, 1), (2, 2), (3, 3), (4, 4)]
tsp_solver = TSPSolver(cities)
iterations = 1000
solution_tour, solution_distance = tsp_solver.hill_climbing(iterations)

# Analysis and Comparison
print("Hill Climbing Solution:")
print("Tour:", solution_tour)
print("Total Distance:", solution_distance)

# Compare with optimal solution (calculated exhaustively for small number of cities)
optimal_tour = min(itertools.permutations(range(len(cities))), key=lambda tour: tsp_solver.total_distance(tour))
optimal_distance = tsp_solver.total_distance(optimal_tour)

print("\nOptimal Solution:")
print("Tour:", optimal_tour)
print("Total Distance:", optimal_distance)

print("\nComparison:")
print("Distance found by Hill Climbing:", solution_distance)
print("Distance of Optimal Solution:", optimal_distance)


# In[15]:


import random
import math

class City:
    def __init__(self, x, y):
        self.x = x
        self.y = y

def generate_random_cities(num_cities):
    return [City(random.randint(0, 100), random.randint(0, 100)) for _ in range(num_cities)]

def distance(city1, city2):
    return math.sqrt((city1.x - city2.x)**2 + (city1.y - city2.y)**2)

def total_distance(route):
    total = 0
    for i in range(len(route)):
        total += distance(route[i], route[(i+1) % len(route)])
    return total

def hill_climbing(initial_route):
    current_route = initial_route[:]
    while True:
        best_distance = total_distance(current_route)
        improved = False
        for i in range(len(current_route)):
            for j in range(i + 1, len(current_route)):
                new_route = current_route[:]
                new_route[i], new_route[j] = new_route[j], new_route[i]
                new_distance = total_distance(new_route)
                if new_distance < best_distance:
                    current_route = new_route
                    best_distance = new_distance
                    improved = True
        if not improved:
            break
    return current_route

def visualize_route(route):
    for city in route:
        print(f"({city.x},{city.y}) -> ", end="")
    print(f"({route[0].x},{route[0].y})")

def main():
    num_cities = 10
    initial_cities = generate_random_cities(num_cities)
    print("Initial Route:")
    visualize_route(initial_cities)

    final_route = hill_climbing(initial_cities)
    print("\nFinal Route after Hill Climbing:")
    visualize_route(final_route)

if __name__ == "__main__":
    main()


# In[ ]:




