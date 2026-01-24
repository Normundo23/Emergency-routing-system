# Intelligent Emergency Routing Algorithm

This document provides a comprehensive breakdown of the core algorithm, mathematical logic, and AI integration used for critical path optimization in the Emergency Routing System.

---

## 1. Core A* Pathfinding Algorithm (Deterministic Layer)

The system utilizes a modified **A*** (A-Star) search algorithm. Unlike standard GPS routing which often minimizes distance, our implementation minimizes **Estimated Time of Arrival (ETA)** by using dynamic edge costs that reflect real-time traffic and predictive risk.

### Operational Workflow
1.  **Priority Queue**: Maintains a "frontier" of nodes to explore, sorted by $f(n) = g(n) + h(n)$.
2.  **$g(n)$ (Actual Cost)**: The total dynamic travel time from the start node to the current node $n$.
3.  **$h(n)$ (Heuristic)**: The estimated time from node $n$ to the goal.
4.  **$f(n)$ (Total Priority)**: The lowest estimated total time to reach the destination via node $n$.

```python
import heapq

def astar(graph, start, goal, cost_fn, heuristic_fn):
    """
    Optimized A* search returning (total_cost, path).
    - cost_fn: Calculates dynamic travel time (Hybrid AI Layer)
    - heuristic_fn: Estimates time to goal (Spatial Layer)
    """
    open_set = [(0.0, start)]  # (f_score, node)
    came_from = {start: None}
    g_score = {start: 0.0}
    closed = set()

    while open_set:
        _, current = heapq.heappop(open_set)
        
        if current in closed: continue
        closed.add(current)

        if current == goal:
            # Reconstruct optimized path
            path = []
            while current is not None:
                path.append(current)
                current = came_from[current]
            return g_score[goal], path[::-1]

        for edge in graph.neighbors(current):
            if edge.blocked: continue
            
            # Hybrid AI Cost Calculation (Dynamic Edge Weight)
            tentative_g = g_score[current] + cost_fn(edge)
            
            if tentative_g < g_score.get(edge.target, float('inf')):
                came_from[edge.target] = current
                g_score[edge.target] = tentative_g
                f_score = tentative_g + heuristic_fn(edge.target)
                heapq.heappush(open_set, (f_score, edge.target))

    raise ValueError("No path found")
```

---

## 2. Dynamic Edge Weight Calculation (Decision Layer)

The "Edge Weight" in our graph is not distance, but **Time**. We calculate this value using a multi-factor formula that accounts for infrastructure, live data, and environmental conditions.

### The Cost Formula
The final cost for an edge is calculated as:
$$ \text{Cost} = \left( T_{\text{base}} \times G \times M_{\text{incident}} \right) $$

#### Components:
1.  **Base Travel Time ($T_{\text{base}}$)**:
    $$ T_{\text{base}} = \frac{\text{length (m)}}{\text{typical speed (m/s)}} + \text{turn penalty (s)} $$
    *   *Note*: Typical speed is derived from OpenStreetMap highway tags (e.g., motorway = 100km/h).

2.  **Global Factor ($G$)**:
    Adjusts speed for environment-wide conditions.
    *   **Weather**: Rain adds a 1.1x multiplier (10% reduction in speed).
    *   **Time of Day**: Peak hours (7 AM - 9 AM, 4 PM - 6 PM) can add up to 1.15x multiplier.

3.  **Incident Multiplier ($M_{\text{incident}}$)**:
    Derived from real-time feeds (TomTom API) or reported accidents.
    *   **Low Severity**: 1.0x (No change)
    *   **Medium Severity**: 1.4x (Roughly 70% of normal speed)
    *   **High Severity**: 1.7x (Roughly 40% of normal speed)
    *   **Road Closure**: 10.0x (or marked as `blocked` to prune from search)

```python
def calculate_edge_cost(edge, context):
    # 1. Base Traversal Time
    speed = edge.live_speed_mps or edge.typical_speed_mps
    base_cost = (edge.length_m / max(speed, 0.1)) + edge.turn_penalty_s
    
    # 2. Global Factors (e.g., Weather)
    global_factor = 1.0
    if context.get("weather") == "rain":
        global_factor = 1.1
        
    # 3. Specific Incident Multipliers
    incident_multiplier = dynamic_penalties.get(id(edge), 1.0)
        
    return base_cost * global_factor * incident_multiplier
```

---

## 3. Fast Spatial Heuristic (Admissible Layer)

To maintain sub-200ms routing performance across large city graphs, we avoid the computationally expensive Haversine formula (spherical trigonometry) in the inner loop. Instead, we use an **Equirectangular Approximation**.

### The Formula
$$ \Delta lat = (lat_1 - lat_2) \times 110,574 $$
$$ \Delta lon = (lon_1 - lon_2) \times 108,000 $$
$$ \text{Distance} = \sqrt{(\Delta lat)^2 + (\Delta lon)^2} $$
$$ H(n) = \frac{\text{Distance}}{\text{Max Possible Speed (30 m/s)}} $$

*   The divisor (30 m/s) ensures the heuristic is **admissible** (it never overestimates the time), which is required for A* to find the mathematically optimal path.

---

## 4. Hybrid AI Layer (Predictive Integration)

The system integrates two specialized Machine Learning models to augment decision-making:

### A. ETA Estimator (ML-Based)
Uses a Random Forest or Gradient Boosted model to predict travel time based on:
-   Edge Length & Historical Speed
-   Hour of Day & Day of Week
-   Real-time Weather & Incident status

### B. Demand/Risk Estimator
Predicts geographical areas with high emergency probability.
-   **Input**: Region ID, Time, Weather.
-   **Output**: Risk Probability (0.0 to 1.0).
-   **Application**: Used to strategically position emergency vehicles *before* incidents occur.
