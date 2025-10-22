import math
def distance(point1, point2):
    return math.sqrt((point2[0] - point1[0])**2 + (point2[1] - point1[1])**2)

def optimize(destinations):
    if not destinations or len(destinations) <= 1:
        return destinations, 0.0

    start_point = destinations[0]
    unvisited = destinations[1:].copy() 
    
    final_path = [start_point]
    current_position = start_point
    total_distance = 0.0

    # Iteratively select the Nearest Neighbor (Greedy Approach)
    while unvisited:
        
        nearest_target = min(
            unvisited, 
            key=lambda point: distance(current_position, point)
        )
        
        # Calculate distance to the chosen target
        min_distance = distance(current_position, nearest_target)
        
        # Move to the nearest target and update state
        final_path.append(nearest_target)
        unvisited.remove(nearest_target)
        current_position = nearest_target
        total_distance += min_distance

    destinations_to_pop = final_path[1:] 

    # Reverse the order of destinations p1 through pn
    destinations_to_pop.reverse()
    final_path_list = [start_point] + destinations_to_pop
    
    return final_path_list, total_distance

# Testing with some inputs
ini_destination = [
    (10, 20),  # Start: p0
    (50, 60),  # Target 1: p1
    (5, 5),    # Target 2: p2
    (80, 10),  # Target 3: p3
    (40, 5)    # Target 4: p4
]

print(f"Initial Destinations (Start: {ini_destination[0]}):")
print(ini_destination)

sorted_list, distance = optimize(ini_destination)

print("\n--- Optimization Results ---")
print(f"Total Minimum Distance (Greedy Estimate): {distance:.2f}")
print("Final Destination List (Ready for 'pop()'):")
print(sorted_list)

# Verify the pop order (p1 -> p2 -> p3 -> p4)
path_for_pop = sorted_list[1:].copy()
pop_order = []
while path_for_pop:
    pop_order.append(path_for_pop.pop())

print(f"\nIntended Traversal Order: {sorted_list[0]} -> {' -> '.join(map(str, pop_order))}")
