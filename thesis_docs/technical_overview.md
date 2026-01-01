# Emergency Routing System: Technical Overview

## 1. System Architecture
The system is a hybrid application with a Python-based intelligent backend and a Capacitor-wrapped web frontend for mobile.

### Backend (Python 3.12+)
- **Core Engine:** `traffic_router.py` - Runs the pathfinding logic.
- **API Framework:** `FastAPI` - Serves the routing endpoints (`/route`, `/route/coords`) (Optional/Embedded).
- **Data Source:** `real_data_fetcher.py` - Fetches live traffic incidents from TomTom API.
- **Graph Processing:** `OSMnx` & `NetworkX` - Downloads and processes OpenStreetMap road networks.
- **Machine Learning:** `scikit-learn` & `joblib` - Used for predictive ETA and Demand modeling.

### Frontend (Html/JS/Capacitor)
- **Framework:** Vanilla HTML/CSS/JS (Lightweight).
- **Mobile Wrapper:** `Capacitor` - Converts the web app into an Android `.apk`.
- **Map Visualization:** `Leaflet.js` (assumed based on standard patterns) or raw canvas rendering.

---

## 2. Algorithms & Logic

### A. Graph Representation
The world is represented as a directed graph $G = (V, E)$.
- **Nodes ($V$):** Intersections/Points from OpenStreetMap.
- **Edges ($E$):** Roads connecting them.
- **Weights:** Each edge has a cost, which is originally Time = Distance / Speed.

### B. Intelligent Cost Calculation
The core "AI" is in how we calculate the cost of an edge *dynamically*.
$$Cost = \frac{Length}{Speed} \times P_{traffic} \times P_{weather} \times P_{closure}$$

1.  **Base Cost:** $\frac{Length_{meters}}{Speed_{mps}}$
2.  **Traffic Penalty ($P_{traffic}$):** Derived from TomTom "Jams" or "Accidents".
    -   *Example:* If an accident is near an edge, speed is reduced by 60% ($Cost \times 2.5$).
3.  **Closure Penalty ($P_{closure}$):** If a road is closed, cost becomes $\infty$.
4.  **Weather Factor ($P_{weather}$):** Global multiplier (e.g., $1.1\times$ for Rain).

### C. Pathfinding (A* Search)
We use the **A* Algorithm** to find the optimal path.
-   **Heuristic ($h(n)$):** We use a simplified Equirectangular distance (faster than Haversine) to estimate distance to the goal.
-   **Optimality:** A* guarantees the shortest path *given the current cost rules*.

---

## 3. The "AI" Components (Machine Learning)
The system is designed to be hybrid. It uses strict rules (above) *plus* trained models.

### A. `ETAEstimator` (`eta_model.joblib`)
-   **Input:** Edge features (Length, Speed Limit), Context (Time of Day, Weather).
-   **Model:** Likely a Random Forest or Gradient Boosted Regressor.
-   **Output:** Predicted travel time (seconds).
-   **Purpose:** Replaces the simple math ($\frac{L}{S}$) with a learned prediction that accounts for complex interaction effects (e.g., "Rush hour on a Friday is worse than Monday").

### B. `DemandEstimator` (`demand_model.joblib`)
-   **Input:** Region ID, Time, Day, Weather.
-   **Output:** Accident Risk Probability (0.0 to 1.0).
-   **Purpose:** Can be used to *pre-position* emergency vehicles in high-risk areas before accidents happen.

---

## 4. Crash Explainer
**The Crash:** `UnboundLocalError: local variable 'blocks' referenced before assignment`
**The Cause:** In `traffic_router.py`, the variable `blocks` was only defined *inside* an `if/else` block. If the code took a different path (e.g., used a cached graph without fetching new traffic), `blocks` was never created.
**The Fix:** We initialized `blocks = set()` and `live_speeds = {}` at the very top of the function to ensure they always exist.
