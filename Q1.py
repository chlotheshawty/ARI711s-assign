import copy
import time

class Grid:
    def __init__(self, x, y):
        self.x = x
        self.y = y

class Environment:
    def __init__(self, width, height):
        self.width = width
        self.height = height
        self.grid = [['clean' for _ in range(width)] for _ in range(height)]  # Initialize grid cells
        self.obstacles = set()

    def add_obstacle(self, x, y):
        self.obstacles.add((x, y))

    def is_obstacle(self, x, y):
        return (x, y) in self.obstacles

    def display(self, robot_position):
        grid_copy = copy.deepcopy(self.grid)
        for obstacle in self.obstacles:
            x, y = obstacle
            grid_copy[y][x] = 'obstacle'
        x, y = robot_position.x, robot_position.y
        grid_copy[y][x] = 'robot'
        
        for row in grid_copy:
            print(' '.join(map(str, row)))

class Robot:
    def __init__(self, x, y, environment):
        self.position = Grid(x, y)
        self.environment = environment
        self.visited = set()
        self.path = []

    def move(self, dx, dy):
        new_x = self.position.x + dx
        new_y = self.position.y + dy

        # Check if new position is within bounds and not an obstacle
        if 0 <= new_x < self.environment.width and 0 <= new_y < self.environment.height \
                and not self.environment.is_obstacle(new_x, new_y):
            self.visited.add((self.position.x, self.position.y))
            self.path.append(Grid(new_x, new_y))
            self.position.x = new_x
            self.position.y = new_y

    def clean(self):
        x, y = self.position.x, self.position.y
        self.environment.grid[y][x] = 'cleaned'

    def backtrack(self):
        if self.path:
            self.path.pop()  # Remove current position
            if self.path:
                last_position = self.path[-1]
                self.position.x = last_position.x
                self.position.y = last_position.y

if __name__ == "__main__":
    env = Environment(6, 10)

    # static obstacles
    env.add_obstacle(3, 3)
    env.add_obstacle(5, 7)
    env.add_obstacle(4, 5)


    # robot initial position
    robot = Robot(1, 1, env)

    # Visualization loop
    while True:
        # Clear robot screen (for better visualization in console)
        print("\033[H\033[J")
        
        env.display(robot.position)
        
        # robot movement
        robot.move(1, 0)  # Move right
        
        # Clean the current position
        robot.clean()

        # Backtrack if necessary
        if robot.position in robot.visited:
            robot.backtrack()

        # Sleep for a short period to slow down visualization
        time.sleep(0.5)
