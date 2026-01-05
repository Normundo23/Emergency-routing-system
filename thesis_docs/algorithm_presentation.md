# Intelligent Emergency Routing Algorithm

This document contains the core algorithm and logic used for critical path optimization in the Emergency Routing System.

---

## 1. Core A* Pathfinding Algorithm (Deterministic Layer)

The system uses a standard A* search algorithm, modified to use dynamic edge costs that reflect real-time traffic and predictive risk.

```python
import heapq

def astar(graph, start, goal, cost_fn, heuristic_fn):
    """
    Standard A* search returning (total_cost, path).
    - cost_fn: Calculates dynamic edge cost (Hybrid AI Layer)
    - heuristic_fn: Estimates time to goal (Spatial Layer)
    """
    open_set = [(0.0, start)]
    came_from = {start: None}
    g_score = {start: 0.0}
    closed = set()

    while open_set:
        _, current = heapq.heappop(open_set)
        
        if current in closed: continue
        closed.add(current)

        if current == goal:
            # Reconstruct path from goal to start
            path = []
            while current is not None:
                path.append(current)
                current = came_from[current]
            return g_score[goal], path[::-1]

        for edge in graph.neighbors(current):
            if edge.blocked: continue
            
            # Hybrid AI Cost Calculation
            tentative_g = g_score[current] + cost_fn(edge)
            
            if tentative_g < g_score.get(edge.target, float('inf')):
                came_from[edge.target] = current
                g_score[edge.target] = tentative_g
                f_score = tentative_g + heuristic_fn(edge.target)
                heapq.heappush(open_set, (f_score, edge.target))

    raise ValueError("No path found")
```

---

## 2. Hybrid AI Cost Function (Decision Layer)

Instead of static distance, edge weights are calculated using a combination of typical speeds, live traffic feeds, and global factors (like weather).

$$ \text{Cost} = \left( \frac{\text{Distance}}{\text{Speed}_{\text{predicted}}} + \text{Turn Penalty} \right) \times \text{Global Factors} \times \text{Incident Penalty} $$

```python
def calculate_edge_cost(edge, context):
    """
    Calculates the 'real-world' cost of traversing an edge.
    """
    # 1. Base Traversal Time (Distance / Speed)
    # Uses 'live_speed' if available from TomTom API, else 'typical_speed'
    speed = edge.live_speed_mps or edge.typical_speed_mps
    base_cost = (edge.length_m / max(speed, 0.1)) + edge.turn_penalty_s
    
    # 2. Global Contextual Factors
    # Example: Heavy rain reduces safe emergency speed by 10%
    global_factor = 1.0
    if context.get("weather") == "rain":
        global_factor = 1.1
        
    # 3. Specific Incident Penalties
    # Penalties applied to edges directly affected by reported accidents
    incident_penalty = 1.0
    if edge.id in dynamic_penalties:
        incident_penalty = dynamic_penalties[edge.id]
        
    return base_cost * global_factor * incident_penalty
```

---

## 3. Fast Spatial Heuristic (Admissible Layer)

To maintain sub-200ms performance, we use an equirectangular approximation for coordinate distance instead of the computationally expensive Haversine formula.

```python
import math

def fast_heuristic(coord1, coord2):
    """
    Optimized heuristic using equirectangular approximation.
    Standardizes lat/lon degrees to meters for quick distance estimation.
    """
    lat1, lon1 = coord1
    lat2, lon2 = coord2
    
    # Approx degrees to meters (Philippines context)
    d_lat = (lat1 - lat2) * 110574.0
    d_lon = (lon1 - lon2) * 108000.0
    
    # Euclidean distance in meters
    distance = math.sqrt(d_lat**2 + d_lon**2)
    
    # Admissible under-estimate: Time = Distance / Max_Possible_Speed (30m/s)
    return distance / 30.0
```
