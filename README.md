# Data-Structure-Project

# Traffic Signal and Emergency Vehicle Handling Simulation

## Overview

This system simulates a city's traffic network, including traffic signal management, road network routing, congestion monitoring, and emergency vehicle routing. The simulation integrates with real-time traffic data, road closures, and vehicle counts to optimize traffic flow and emergency vehicle handling.

The system provides a number of functionalities, including:
- Displaying the city’s traffic network
- Managing traffic signals and their statuses
- Handling emergency vehicle routing
- Simulating vehicle routing
- Updating road networks based on closures and accidents

## Features

- **Road Network Representation**: The city is modeled as a graph of intersections connected by roads. Each road has a travel time (weight), and roads are bi-directional.
- **Traffic Signal Management**: Simulates traffic signals at intersections, adjusting green times based on vehicle counts.
- **Emergency Vehicle Routing**: Finds optimal routes for emergency vehicles, overriding traffic signals to ensure faster passage.
- **Vehicle Count Monitoring**: Keeps track of vehicle counts at each intersection to simulate traffic congestion and adjust signal timings.
- **Road Closure Simulation**: Simulates road closures and their effects on traffic routing.
- **Dijkstra’s Algorithm**: Computes the shortest path between two intersections in terms of travel time.
- **Congestion Monitoring**: Displays traffic congestion status based on the vehicle counts on different roads.

## Data Files

The system uses several CSV files to represent the road network, vehicle counts, traffic signals, and road closures. Below are the expected file formats:

1. **road_network.csv**:
   - Columns: `From Intersection, To Intersection, Travel Time`
   - Describes the roads between intersections with their corresponding travel times.

2. **traffic_signals.csv**:
   - Columns: `Intersection, Signal Time`
   - Contains the green light time for each intersection.

3. **vehicles.csv**:
   - Columns: `Vehicle ID, Road (start -> end)`
   - Lists vehicles and the roads they are traveling on.

4. **road_closures.csv**:
   - Columns: `From Intersection, To Intersection, Status (Blocked/Clear)`
   - Simulates road closures (either blocked or cleared).

5. **emergency_vehicles.csv**:
   - Columns: `Vehicle ID, Start Intersection, End Intersection`
   - Contains the start and end points of emergency vehicles that require prioritized routing.

## Compilation and Execution

### Prerequisites:
- C++ Compiler (e.g., g++ or clang++)
- C++11 or later standard

### Compilation Instructions:

To compile the code, use the following command:

```bash
g++ -std=c++11 -o TrafficSimulation main.cpp
```

### Running the Program:

After compilation, run the program with the following command:

```bash
./TrafficSimulation
```

The program will prompt you to choose various options to simulate traffic signal management, road closures, and emergency vehicle routing.

### Available Operations:
1. **Display the City Traffic Network (Initial)**: Show the initial road network loaded from `road_network.csv`.
2. **Display Traffic Signal Status**: Display the current state of all traffic signals.
3. **Display Congestion Status**: Display vehicle congestion for each road.
4. **Display Blocked Roads**: List all roads that are currently blocked due to accidents or closures.
5. **Handle Emergency Vehicle Routing**: Compute the optimal path for an emergency vehicle between two intersections.
6. **Block Roads due to Accident**: Simulate the blocking of a road due to an accident.
7. **Simulate Vehicle Routing**: Display all possible paths for vehicles between two intersections.
8. **Display the City Network of Working Roads**: Show the current active road network after road closures.
9. **Handle Emergency Vehicle Routing for CSV file**: Simulate routing for emergency vehicles from `emergency_vehicles.csv`.
10. **Apply Dijkstra's Algorithm**: Find the shortest path between two intersections using Dijkstra’s algorithm.

### Example Output:

```bash
-------Simulation of Traffic Signals and Emergency Vehicle Handling-------
1. Display the City Traffic Network (Initial)
2. Display Traffic Signal Status
3. Display Congestion Status
4. Display Blocked Roads
5. Handle Emergency Vehicle Routing
6. Block Roads due to Accident
7. Simulate Vehicle Routing
8. Display the City Network of Working Roads
9. Handle Emergency Vehicle Routing for CSV file
10. Apply Dijkstra's Algorithm
11. Exit
Enter your choice: 1
Initial Road Network:
A ->  (B, 5)  (C, 10)  
B ->  (A, 5)  (C, 3)  
...
```

## Data Structures

1. **RoadNetwork Class**: Represents the road network, including methods for adding/removing roads and intersections, and for running Dijkstra's algorithm.
2. **TrafficSignals Class**: Handles traffic signals, including adding new signals, updating signal times, and displaying traffic signal statuses.
3. **EmergencyVehicleHandling Class**: Handles the routing of emergency vehicles, overrides traffic signal timings, and calculates the shortest path using a heuristic.
4. **VehicleHashTable Class**: A hash table that tracks vehicle counts on roads, used for congestion monitoring.
5. **BlockedRoadStack Class**: A stack data structure to store blocked roads due to accidents.
6. **RoadSignals Class**: Represents an individual road signal and its associated properties (signal time, vehicle count).
7. **Travel_time and IntersectionNode Structs**: Used to model roads (edges) and intersections (nodes) in the road network.

## Memory Management

The program uses dynamic memory allocation for various data structures like `IntersectionNode`, `Travel_time`, `HashNode`, and `RoadSignals`. Proper memory deallocation is ensured in the destructors of these classes to prevent memory leaks.

## Known Limitations

- The program currently assumes that all intersections are labeled from 'A' to 'Z', which may not be suitable for larger networks.
- The simulation does not yet handle real-time updates to vehicle counts or traffic conditions; all data is pre-loaded from CSV files.
- Emergency vehicle routing uses a simplified heuristic (0) for A* search, and this could be enhanced with a more realistic heuristic.

## Future Improvements

- Real-time data integration for vehicle counts, traffic conditions, and road closures.
- More sophisticated routing algorithms that take traffic signals and congestion into account dynamically.
- A user interface (UI) to visualize the road network and traffic conditions in real-time.
