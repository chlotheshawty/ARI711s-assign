#Modelling the Environment

class Environment:
    def __init__(self, width, height, obstacles):
        self.width = width
        self.height = height
        self.obstacles = obstacles
        self.grid = [[0 for _ in range(width)] for _ in range(height)]
        for obstacle in obstacles:
            x, y = obstacle
            self.grid[y][x] = 1
    
    def is_obstacle(self, x, y):
        return self.grid[y][x] == 1

class Robot:
    def __init__(self, environment, start_position):
        self.environment = environment
        self.position = start_position
    
    def is_valid_move(self, x, y):
        return (0 <= x < self.environment.width) and (0 <= y < self.environment.height) and not self.environment.is_obstacle(x, y)
    
    def move_left(self):
        new_x = self.position[0] - 1
        new_y = self.position[1]
        if self.is_valid_move(new_x, new_y):
            self.position = (new_x, new_y)
    
    def move_right(self):
        new_x = self.position[0] + 1
        new_y = self.position[1]
        if self.is_valid_move(new_x, new_y):
            self.position = (new_x, new_y)

width = 10
height = 10
obstacles = [(3, 3), (4, 5), (7, 8)]
env = Environment(width, height, obstacles)
start_position = (1, 1) 
robot = Robot(env, start_position)

#Cost Function and Heuristic

import heapq
import math

def simple_cost_function(action):
    return 1

def simple_heuristic(current, target):
    return abs(current[0] - target[0]) + abs(current[1] - target[1])

def sophisticated_heuristic(current, target, environment):
    dx = abs(current[0] - target[0])
    dy = abs(current[1] - target[1])
    heuristic_base = math.sqrt(dx**2 + dy**2)
    
    penalty = 0
    for x in range(min(current[0], target[0]), max(current[0], target[0]) + 1):
        for y in range(min(current[1], target[1]), max(current[1], target[1]) + 1):
            if environment.is_obstacle(x, y):
                penalty += 1
    
    return heuristic_base + penalty

start_position = (1, 1)
target_position = (8, 8)
robot = Robot(env, start_position)
cost = simple_cost_function((1, 0))
print("Cost for moving right:", cost)

heuristic_value = simple_heuristic((1, 1), (8, 8))
print("Simple Heuristic:", heuristic_value)
sophisticated_heuristic_value = sophisticated_heuristic((1, 1), (8, 8), env)
print("Sophisticated Heuristic:", sophisticated_heuristic_value)

class Terrain:
    CARPET = 1
    HARDWOOD = 2
    TILE = 3

def energy_expenditure(terrain):
    if terrain == Terrain.CARPET:
        return 1.2
    elif terrain == Terrain.HARDWOOD:
        return 1.0
    elif terrain == Terrain.TILE:
        return 1.1
    else:
        return 1.0

def terrain_cost(terrain):
    if terrain == Terrain.CARPET:
        return 2
    elif terrain == Terrain.HARDWOOD:
        return 1
    elif terrain == Terrain.TILE:
        return 1
    else:
        return 1 

def sophisticated_cost_function(action, terrain):
    terrain_cost_value = terrain_cost(terrain)
    return terrain_cost_value * energy_expenditure(terrain)

def sophisticated_heuristic(current, target, environment, terrain):
    dx = abs(current[0] - target[0])
    dy = abs(current[1] - target[1])
    heuristic_base = math.sqrt(dx**2 + dy**2)

    penalty = 0
    for x in range(min(current[0], target[0]), max(current[0], target[0]) + 1):
        for y in range(min(current[1], target[1]), max(current[1], target[1]) + 1):
            if environment.is_obstacle(x, y):
                penalty += 1 
    
    return heuristic_base + penalty * energy_expenditure(terrain)

start_position = (1, 1)
target_position = (8, 8)
terrain = Terrain.HARDWOOD
robot = Robot(env, start_position)

cost = sophisticated_cost_function((1, 0), terrain)
print("Cost for moving right on hardwood:", cost)

heuristic_value = sophisticated_heuristic((1, 1), (8, 8), env, terrain)
print("Sophisticated Heuristic on hardwood:", heuristic_value)

#A Implementation and Testing

class Node:
    def __init__(self, position, g_cost, h_cost):
        self.position = position
        self.g_cost = g_cost
        self.h_cost = h_cost
        self.f_cost = g_cost + h_cost
        self.parent = None

def astar_search(env, start, target, terrain):
    open_list = []
    closed_list = set()

    start_node = Node(start, 0, simple_heuristic(start, target))
    target_node = Node(target, 0, 0)
    
    heapq.heappush(open_list, (start_node.f_cost, start_node))
    
    while open_list:
        current_node = heapq.heappop(open_list)[1]
        
        if current_node.position == target:
            path = []
            while current_node:
                path.append(current_node.position)
                current_node = current_node.parent
            path.reverse()
            return path
        
        closed_list.add(current_node.position)
        
        for action in [(1, 0), (-1, 0), (0, 1), (0, -1)]:
            new_position = (current_node.position[0] + action[0], current_node.position[1] + action[1])
            
            if (new_position[0] < 0 or new_position[0] >= env.width or
                new_position[1] < 0 or new_position[1] >= env.height or
                env.is_obstacle(*new_position) or
                new_position in closed_list):
                continue
            
            new_g_cost = current_node.g_cost + sophisticated_cost_function(action, terrain)
            new_h_cost = sophisticated_heuristic(new_position, target, env, terrain)
            new_f_cost = new_g_cost + new_h_cost
            
            neighbor_node = None
            for _, node in open_list:
                if node.position == new_position:
                    neighbor_node = node
                    break
            
            if neighbor_node is None or new_f_cost < neighbor_node.f_cost:
                neighbor_node = Node(new_position, new_g_cost, new_h_cost)
                neighbor_node.parent = current_node
                heapq.heappush(open_list, (neighbor_node.f_cost, neighbor_node))
    
    return None

start_position = (1, 1)
target_position = (8, 8)
terrain = Terrain.HARDWOOD
robot = Robot(env, start_position)

path = astar_search(env, start_position, target_position, terrain)
print("Optimal Path:", path)

