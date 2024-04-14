

class TSP:
    def __init__(self, places, distances):
        self.places = places
        self.distances = distances

    def calculate_total_distance(self, route):
        total_distance = 0
        for i in range(len(route) - 1):
            total_distance += self.distances[route[i]][route[i + 1]]
        total_distance += self.distances[route[-1]][route[0]]
        return total_distance

places = ['Dorado Park', 'Khomasdal', 'Katutura', 'Eros', 'Klein Windhoek']
distances = [
    [0, 7, 20, 15, 12],
    [10, 0, 6, 14, 18],
    [20, 6, 0, 15, 30],
    [15, 14, 25, 0, 2],
    [12, 18, 30, 2, 0]
]

tsp = TSP(places, distances)
route = [0, 1, 2, 3, 4]
total_distance = tsp.calculate_total_distance(route)
print("Total distance:", total_distance)


import random

def generate_initial_route(num_places):
    return random.sample(range(num_places), num_places)

def explore_neighbours(route):
    i, j = random.sample(range(len(route)), 2)
    new_route = route[:]
    new_route[i], new_route[j] = new_route[j], new_route[i]
    return new_route

def hill_climbing(tsp):
    current_route = generate_initial_route(len(tsp.places))
    current_distance = tsp.calculate_total_distance(current_route)
    
    while True:
        neighbours = [explore_neighbours(current_route) for _ in range(len(tsp.places)**2)]
        best_neighbour = min(neighbours, key=lambda route: tsp.calculate_total_distance(route))
        best_neighbour_distance = tsp.calculate_total_distance(best_neighbour)
        
        if best_neighbour_distance < current_distance:
            current_route = best_neighbour
            current_distance = best_neighbour_distance
        else:
            break
            
    return current_route, current_distance

optimal_route, optimal_distance = hill_climbing(tsp)
print("Optimal route:", optimal_route)
print("Optimal distance:", optimal_distance)


import matplotlib.pyplot as plt

def plot_route(tsp, route):
    x = [tsp.places[i] for i in route]
    x.append(x[0])
    y = [tsp.places[i] for i in route]
    y.append(y[0])
    
    plt.plot(x, y, marker='o')
    plt.xlabel('Places')
    plt.ylabel('Places')
    plt.title('Travelling Salesman Problem')
    plt.grid(True)
    plt.show()

plot_route(tsp, optimal_route)

