*This project has been created as part of the 42 curriculum by dkhmaruk.*

# Fly-in

## Description

Fly-in is a Python-based drone routing and simulation project.

The goal is to transport a specified number of drones from a starting zone to a destination zone through a network of interconnected zones, while respecting movement costs, zone occupancy limits, and connection capacities.

The project uses a custom graph implementation, pathfinding algorithms, capacity-aware scheduling, and simulation-based route optimization.

A graphical interface built with Pygame visualizes the drone movements and allows users to inspect the simulation step by step.

## Instructions

### Requirements

- Python 3.10 or newer
- pip
- Make

### Installation

Install the project dependencies:

```bash
make install
```

### Running the program

Run the graphical simulator with the default map:

```bash
make run
```

Run a specific map:

```bash
make run MAP=maps/challenger/01_the_impossible_dream.txt
```

Alternatively, run the program directly:

```bash
python3 main.py maps/test_map.txt
```

### Makefile commands

| Command | Description |
|---|---|
| `make install` | Install the required dependencies |
| `make run` | Launch the simulation |
| `make debug` | Run the application with the Python debugger |
| `make clean` | Remove temporary files and Python caches |
| `make lint` | Run flake8 and mypy |

### GUI Controls

| Key / Action | Function |
|---|---|
| Space | Pause or resume the simulation |
| Up arrow | Increase animation speed |
| Down arrow | Decrease animation speed |
| Right arrow | Jump to the next turn |
| Left arrow | Return to the previous turn |
| R | Restart the simulation |
| F | Fit the entire map into the window |
| Mouse wheel | Zoom in or out |
| Middle / right mouse drag | Move the map |
| Esc | Exit the application |

The GUI window can also be resized by dragging its borders.

## Algorithm Explanation

### Graph Representation

The network is represented using a custom, object-oriented graph implementation.

Each zone is represented by a `Zone` object and each connection by a `Connection` object.

The `Graph` class stores:

- A dictionary of zones.
- An adjacency list for efficient neighbor lookup.
- A dictionary of connections, including their capacities.

Undirected connections are identified using `frozenset`, so the connections A–B and B–A are treated as identical.

No external graph or pathfinding libraries are used.

### Pathfinding — Dijkstra's Algorithm

The `Pathfinder` class uses Dijkstra's algorithm to find routes from the starting zone to the destination.

Movement costs depend on the destination zone:

- Normal zones: 1 turn.
- Priority zones: 1 turn.
- Restricted zones: 2 turns.
- Blocked zones: inaccessible.

When two routes have the same movement cost, routes containing more priority zones are preferred.

The implementation uses Python's `heapq` module to manage the priority queue.

Dijkstra's algorithm has a typical time complexity of O((V + E) log V) for a graph with V vertices and E edges using a binary heap.

### Finding Multiple Routes

The Pathfinder searches for multiple routes between the start and end zones.

It initially searches for internally disjoint paths and can also consider alternative routes sharing some zones or connections, subject to capacity-related checks.

Alternative routes allow drones to travel through different parts of the network instead of relying on a single path.

### Drone Scheduling

The `Scheduler` class assigns drones to available routes.

Assignments consider:

- Route movement cost.
- The number of drones assigned to each route.
- Estimated route capacity based on zone and connection capacities.

This reduces unnecessary congestion and provides an initial allocation for further optimization.

### Simulation

The `Simulation` class performs turn-based drone movement.

Each turn consists of planning and applying simultaneous movements.

The simulation considers:

- Zone occupancy limits.
- Connection capacity limits.
- Restricted-zone transitions.
- Simultaneous departures and arrivals.
- Circular movement dependencies.
- Future capacity reservations.

A `SimulationValidator` checks planned movements and resulting drone states.

### Route Optimization

The `RouteOptimizer` class improves route allocation through trial simulations.

It:

1. Generates an initial routing solution.
2. Searches for alternative routes.
3. Evaluates different drone distributions.
4. Simulates candidate allocations.
5. Selects the allocation with the lowest observed turn count.

Trial simulations are performed without opening the graphical interface.

The optimizer is heuristic and does not guarantee a globally optimal solution for every possible graph.

## Visual Representation

Fly-in includes a graphical simulation interface implemented with Pygame.

The GUI displays:

- Zones positioned according to their map coordinates.
- Connections between zones.
- Animated drones with individual identifiers.
- Different zone types and custom map colors.
- Drone positions during restricted transitions.
- A delivered-drone counter.
- Turn number and animation speed.

Multiple drones occupying the same zone are displayed with separate visual offsets to prevent overlap.

The interface also supports zooming, panning, window resizing, restarting, pausing, and step-by-step inspection.

These features make it easier to understand routing decisions, congestion, and simultaneous drone movements.

## Input Format

Maps are provided as text files.

A map specifies the number of drones, zones, connections, and optional metadata.

Example:

```text
nb_drones: 2

start_hub: Start 0 0
hub: A 1 0 [zone=normal color=blue max_drones=1]
end_hub: End 2 0

connection: Start-A
connection: A-End
```

Supported zone types are `normal`, `priority`, `restricted`, and `blocked`.

Zone metadata can define `zone`, `color`, and `max_drones`.

Connection metadata can define `max_link_capacity`.

Comments and blank lines are ignored.

## Example Output

For the example above, a possible turn-by-turn movement sequence is:

```text
D1-A
D1-End D2-A
D2-End
```

Each line represents one simulation turn.

The graphical interface visualizes the same movements.

## Resources

The following resources were used for learning, implementation, and debugging:

- 42 Fly-in project subject and evaluation guidelines.
- Python official documentation: https://docs.python.org/3/
- Python `heapq` documentation: https://docs.python.org/3/library/heapq.html
- Pygame documentation: https://www.pygame.org/docs/

### AI Usage

AI was used as a development assistance tool for:

- Discussing the project architecture and algorithmic approaches.
- Explaining Python concepts and pathfinding techniques.
- Suggesting implementation changes and optimizations.
- Assisting with debugging and test scenarios.
- Reviewing code organization and documentation.