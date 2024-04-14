import random

class City:
    def __init__(self, name, x, y):
        self.name = name
        self.x = x
        self.y = y

def calculate_distance(city1, city2):
    return ((city1.x - city2.x) ** 2 + (city1.y - city2.y) ** 2) ** 0.5

def total_distance(route, cities):
    total = 0
    for i in range(len(route)):
        total += calculate_distance(cities[route[i]], cities[route[(i + 1) % len(route)]])
    return total

def generate_random_route(num_cities):
    return random.sample(range(num_cities), num_cities)

def hill_climbing_tsp(cities, max_iterations=1000):
    num_cities = len(cities)
    current_route = generate_random_route(num_cities)
    current_distance = total_distance(current_route, cities)

    for _ in range(max_iterations):
        neighbor_route = current_route.copy()
        i, j = random.sample(range(num_cities), 2)
        neighbor_route[i], neighbor_route[j] = neighbor_route[j], neighbor_route[i]
        neighbor_distance = total_distance(neighbor_route, cities)

        if neighbor_distance < current_distance:
            current_route = neighbor_route
            current_distance = neighbor_distance

    return current_route

def visualize_route(route, cities):
    for i in range(len(route)):
        print(f"{cities[route[i]].name} -> ", end="")
    print(f"{cities[route[0]].name}")

# Example usage
if __name__ == "__main__":
    # Create some sample cities
    city_names = ["A", "B", "C", "D", "E"]
    city_coordinates = [(0, 0), (1, 2), (3, 1), (4, 3), (2, 4)]
    cities = [City(name, x, y) for name, (x, y) in zip(city_names, city_coordinates)]

    # Solve TSP using hill climbing
    best_route = hill_climbing_tsp(cities)
    print("Best route found:")
    visualize_route(best_route, cities)
    print(f"Total distance: {total_distance(best_route, cities)}")