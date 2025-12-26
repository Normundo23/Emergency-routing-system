"""
Minimal real-time emergency routing prototype.

Key pieces:
- Graph representation with live traffic/incidents.
- ETA "model" placeholder (can swap in ML model).
- A* search using traffic-aware edge costs and admissible heuristic.
- Reroute loop that reacts to feed changes.

Designed to be small and hackable; replace feed/model stubs with real systems.
"""
from __future__ import annotations

import argparse
import heapq
import math
import os
import sys
import time
import warnings
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Callable, Dict, Iterable, List, Optional, Set, Tuple

import joblib
import pandas as pd
from scipy.spatial import cKDTree
from real_data_fetcher import RealDataFetcher

# --- Configuration ---
# Fallback to hardcoded key if env var is missing (User provided)
DEFAULT_KEY = "YJ7ckNeXM9KeiCuC7DYuqo6JWvBOzgDl" 
TOMTOM_API_KEY = os.environ.get("TOMTOM_API_KEY") or os.environ.get("TOMTOM_KEY") or DEFAULT_KEY
fetcher = RealDataFetcher(TOMTOM_API_KEY) if TOMTOM_API_KEY else None

# Optional heavy deps: used in loader/api if installed.
try:  # pragma: no cover - optional
    import osmnx as ox  # type: ignore
except ImportError:  # pragma: no cover
    ox = None  # lazy guard below

try:  # pragma: no cover - optional
    from fastapi import FastAPI
    from fastapi.responses import FileResponse
    from fastapi.staticfiles import StaticFiles
    from pydantic import BaseModel
except ImportError:  # pragma: no cover
    FastAPI = None
    BaseModel = object  # type: ignore


@dataclass
class Edge:
    target: str
    length_m: float
    typical_speed_mps: float
    turn_penalty_s: float = 0.0
    live_speed_mps: Optional[float] = None
    blocked: bool = False
    geometry: List[Tuple[float, float]] = field(default_factory=list) # List of (lat, lon)

    def live_eta(self) -> float:
        """Return estimated travel time for this edge using live speed if present."""
        speed = self.live_speed_mps or self.typical_speed_mps
        speed = max(speed, 0.1)  # avoid division by zero
        return self.length_m / speed + self.turn_penalty_s


@dataclass
class Graph:
    adjacency: Dict[str, List[Edge]] = field(default_factory=dict)
    coordinates: Dict[str, Tuple[float, float]] = field(default_factory=dict)  # optional, for heuristic
    source: str = "unknown"
    _kdtree: Optional[cKDTree] = None
    _node_list: List[str] = field(default_factory=list)

    def build_kdtree(self):
        """Builds KDTree for fast nearest-node lookup."""
        if not self.coordinates:
            return
        self._node_list = list(self.coordinates.keys())
        # cKDTree expects (x, y) -> (lon, lat) usually, but we stored (lat, lon)
        # Let's standardize on (lat, lon) for the tree to match our input queries
        coords = [self.coordinates[n] for n in self._node_list]
        self._kdtree = cKDTree(coords)

    def add_edge(
        self,
        source: str,
        target: str,
        length_m: float,
        typical_speed_mps: float,
        turn_penalty_s: float = 0.0,
        geometry: List[Tuple[float, float]] = None,
    ) -> None:
        self.adjacency.setdefault(source, []).append(
            Edge(
                target=target,
                length_m=length_m,
                typical_speed_mps=typical_speed_mps,
                turn_penalty_s=turn_penalty_s,
                geometry=geometry or []
            )
        )

    def neighbors(self, node: str) -> Iterable[Edge]:
        return self.adjacency.get(node, [])


def haversine_m(src: Tuple[float, float], dst: Tuple[float, float]) -> float:
    """Compute approximate distance between two lat/lon coordinates in meters."""
    lat1, lon1 = src
    lat2, lon2 = dst
    R = 6371000
    phi1 = math.radians(lat1)
    phi2 = math.radians(lat2)
    dphi = math.radians(lat2 - lat1)
    dlambda = math.radians(lon2 - lon1)
    a = math.sin(dphi / 2) ** 2 + math.cos(phi1) * math.cos(phi2) * math.sin(dlambda / 2) ** 2
    return 2 * R * math.atan2(math.sqrt(a), math.sqrt(1 - a))


def fast_heuristic(graph: Graph, node: str, goal: str) -> float:
    """
    Optimized heuristic using equirectangular approximation.
    Much faster than Haversine for A* inner loop.
    """
    if node not in graph.coordinates or goal not in graph.coordinates:
        return 0.0
    
    lat1, lon1 = graph.coordinates[node]
    lat2, lon2 = graph.coordinates[goal]
    
    # Approx degrees to meters (at ~14 deg lat)
    # 1 deg lat ~ 110.574 km
    # 1 deg lon ~ 111.320 * cos(lat) km => at 14N: ~108 km
    
    d_lat = (lat1 - lat2) * 110574.0
    d_lon = (lon1 - lon2) * 108000.0 # Approximate for Philippines
    
    dist_sq = d_lat*d_lat + d_lon*d_lon
    
    # We want time = dist / max_speed
    # Avoid sqrt if we can, but A* needs valid admissible heuristic.
    # dist = math.sqrt(dist_sq)
    # Using 30m/s (108km/h) as max speed
    return math.sqrt(dist_sq) / 30.0


def astar(
    graph: Graph,
    start: str,
    goal: str,
    cost_fn: Callable[[Edge], float],
    heuristic_fn: Callable[[str], float],
) -> Tuple[float, List[str], int]:
    """
    Standard A* search returning (total_cost, path, nodes_visited).
    Raises ValueError if no path exists.
    """
    open_set: List[Tuple[float, str]] = []
    heapq.heappush(open_set, (0.0, start))
    came_from: Dict[str, Optional[str]] = {start: None}
    g_score: Dict[str, float] = {start: 0.0}
    closed: Set[str] = set()
    nodes_visited = 0

    while open_set:
        _, current = heapq.heappop(open_set)
        
        if current in closed:
            continue
        closed.add(current)
        nodes_visited += 1

        if current == goal:
            # reconstruct path
            path = []
            node = current
            while node is not None:
                path.append(node)
                node = came_from[node]
            path.reverse()
            return g_score[current], path, nodes_visited

        for edge in graph.neighbors(current):
            if edge.blocked:
                continue
            tentative_g = g_score[current] + cost_fn(edge)
            if tentative_g < g_score.get(edge.target, math.inf):
                came_from[edge.target] = current
                g_score[edge.target] = tentative_g
                f_score = tentative_g + heuristic_fn(edge.target)
                heapq.heappush(open_set, (f_score, edge.target))

    raise ValueError("No path found")











# --- OSM import utility -------------------------------------------------------
def load_graph_from_osm(place: Optional[str] = None, north: float = None, south: float = None, east: float = None, west: float = None, point: Tuple[float, float] = None, dist: int = 5000) -> Graph:
    """
    Load a drivable graph from OSM using osmnx.
    Prioritizes (point, dist) for fixed radius loading.
    Falls back to bounding box or place name.
    """
    if ox is None:
        raise ImportError("osmnx is required for OSM loading. Install with `pip install osmnx`.")
    
    # Inclusive filter for all drivable roads including private/service
    # "drive" excludes some service roads. We want everything.
    # Note: custom_filter overwrites network_type logic roughly.
    inclusive_filter = '["highway"~"motorway|trunk|primary|secondary|tertiary|residential|unclassified|service|living_street"]'
    
    # 1. Radius Mode (Preferred)
    if point is not None:
        print(f"Downloading graph from point {point} with radius {dist}m...", flush=True)
        G = ox.graph_from_point(point, dist=dist, custom_filter=inclusive_filter, simplify=False)
    
    # 2. Place Mode
    elif place:
        print(f"Downloading graph for place: {place}...", flush=True)
        G = ox.graph_from_place(place, custom_filter=inclusive_filter, simplify=False)
    
    # 3. BBox Mode
    elif None not in (north, south, east, west):
        print(f"Downloading graph from bbox...", flush=True)
        # osmnx 1.x uses bbox=(north, south, east, west) as keyword argument
        G = ox.graph_from_bbox(bbox=(north, south, east, west), custom_filter=inclusive_filter, simplify=False)
             
    else:
        raise ValueError("Provide point+dist, place, or bounding box.")

    # Strict Connectivity Check
    # Ensure we only keep the largest connected component to avoid "island" roads
    if len(G) > 0:
        # ox 1.x vs 0.x compatibility
        try:
             G = ox.utils_graph.get_largest_component(G, strongly=True)
        except AttributeError:
             # Fallback for older osmnx
             # G = max(nx.strongly_connected_components(G), key=len)
             pass 
             
    g = Graph()
    g.source = "osm"
    # store coordinates
    for node, data in G.nodes(data=True):
        g.coordinates[str(node)] = (data.get("y"), data.get("x"))
    
    # add edges
    # OSMnx MultiDiGraph edges are (u, v, key, data)
    for u, v, key, data in G.edges(keys=True, data=True):
        try:
             # Handle list or single value
            val = data.get("length", 1.0)
            if isinstance(val, list):
                val = val[0]
            length = float(val)
        except (ValueError, TypeError):
             length = 50.0 # Default fallback
        
        # Parse speed
        maxspeed = data.get("maxspeed", None)
        highway = data.get("highway", None)
        speed_kph = 40.0 # Safer default

        # Heuristic based on highway type/hierarchy
        if highway:
            if isinstance(highway, list): highway = highway[0]
            if highway in ["motorway", "motorway_link", "trunk"]:
                speed_kph = 100.0
            elif highway in ["primary", "primary_link"]:
                speed_kph = 60.0 # Urban primary
            elif highway in ["secondary", "secondary_link"]:
                speed_kph = 50.0
            elif highway in ["tertiary", "tertiary_link"]:
                speed_kph = 40.0
            else:
                speed_kph = 30.0 # residential etc

        # Override with explicit maxspeed if valid
        if maxspeed:
            if isinstance(maxspeed, list): maxspeed = maxspeed[0]
            try:
                # Clean string "50 mph" -> 50
                clean_speed = str(maxspeed).lower().replace("mph","").replace("kph","").replace("km/h","").strip()
                val = float(clean_speed)
                speed_kph = val
            except (ValueError, TypeError):
                pass
            
        typical_speed_mps = max(speed_kph * (1000.0 / 3600.0), 1.0)
        # With simplify=False, we have many short edges.
        # Adding a penalty per edge would break routing (1km road = 50 edges = 100s penalty).
        turn_penalty = 0.0 
        
        # Extract geometry (curved roads)
        edge_geom = []
        if "geometry" in data:
            # data["geometry"] is a Shapely LineString. coords is list of (lon, lat).
            # We need (lat, lon).
            line_geom = data["geometry"]
            try:
                # Handle Shapely LineString
                if hasattr(line_geom, "coords"):
                     edge_geom = [(lat, lon) for lon, lat in line_geom.coords]
                # Handle raw list if simplified differently
                elif isinstance(line_geom, list):
                     edge_geom = [(p[1], p[0]) for p in line_geom]
                # if len(edge_geom) > 2:
                #      print(f"DEBUG: Captured {len(edge_geom)} points for edge {u}->{v}", flush=True)
            except Exception as e:
                # print(f"DEBUG: Geom parse fail for edge {u}->{v}: {e}", flush=True)
                pass
        else:
            # print(f"DEBUG: No geometry for edge {u}->{v}", flush=True)
            pass
        
        # Add u -> v
        g.add_edge(str(u), str(v), length_m=length, typical_speed_mps=typical_speed_mps, turn_penalty_s=turn_penalty, geometry=edge_geom)
        
        # Check if two-way
        oneway = data.get("oneway", False)
        is_oneway = False
        if oneway is True or oneway == "yes" or (isinstance(oneway, list) and "yes" in oneway):
             is_oneway = True
             
        if not is_oneway:
            # If two-way, add v -> u
            # Reverse geometry if present
            rev_geom = edge_geom[::-1] if edge_geom else []
            g.add_edge(str(v), str(u), length_m=length, typical_speed_mps=typical_speed_mps, turn_penalty_s=turn_penalty, geometry=rev_geom)
            
    return g

# --- Helpers ---
CACHE_DIR = Path("graph_cache")
CACHE_DIR.mkdir(exist_ok=True)



def download_graph_with_timeout(lat: float, lon: float, radius: int, timeout: int = 30) -> Optional[Graph]:
    """Downloads graph using the existing robust loader."""
    try:
        # Correctly call the complex signature with kwargs
        return load_graph_from_osm(point=(lat, lon), dist=radius)
    except Exception as e:
        try:
            with open("download_error.log", "w") as f:
                f.write(f"Error: {e}\n")
                import traceback
                traceback.print_exc(file=f)
        except: pass
        print(f"Download failed: {e}", flush=True)
        return None

    if graph.source == "osm" and graph._kdtree is None:
         graph.build_kdtree()
    return graph

def save_graph_cache(graph: Graph, path: Path):
    joblib.dump(graph, path)

def load_graph_cache(path: Path) -> Graph:
    return joblib.load(path)


def nearest_node(graph: Graph, lat: float, lon: float) -> str:
    """
    Find nearest node. Uses KDTree if available, else O(N) fallback.
    """
    if graph._kdtree:
        # Query nearest neighbor (k=1)
        dist, idx = graph._kdtree.query((lat, lon))
        return graph._node_list[idx]

    # Fallback legacy linear scan
    best_id = None
    best_d = math.inf
    for node_id, (nlat, nlon) in graph.coordinates.items():
        d = haversine_m((lat, lon), (nlat, nlon))
        if d < best_d:
            best_d = d
            best_id = node_id
    if best_id is None:
        raise ValueError("Graph has no coordinates to snap to.")
    return best_id


def path_to_coords(graph: Graph, path: List[str]) -> List[Dict[str, float]]:
    coords = []
    if not path:
        return coords
        
    # Start point
    if path[0] in graph.coordinates:
        lat, lon = graph.coordinates[path[0]]
        coords.append({"lat": lat, "lon": lon})
        
    for i in range(len(path) - 1):
        u, v = path[i], path[i+1]
        
        # Find edge u->v
        edge = None
        for e in graph.neighbors(u):
            if e.target == v:
                edge = e
                break
        
        if edge and edge.geometry:
            # Append detailed geometry logic
            # Excluding start point if it's typically close to u, 
            # but for safety just append all, filtering duplicates if needed?
            # Geometry usually includes u and v coords or close to them.
            # Let's just append the middle points or all points.
            for lat_pt, lon_pt in edge.geometry:
                coords.append({"lat": lat_pt, "lon": lon_pt})
        elif v in graph.coordinates:
            lat, lon = graph.coordinates[v]
            coords.append({"lat": lat, "lon": lon})
            
    return coords

def steps_from_coords(coords: List[Dict[str, float]]) -> List[Dict[str, Any]]:
    steps: List[Dict[str, Any]] = []
    if not coords:
        return steps
    prev = coords[0]
    total = 0.0
    for cur in coords[1:]:
        d = haversine_m((prev["lat"], prev["lon"]), (cur["lat"], cur["lon"]))
        total += d
        steps.append({"instruction": "Proceed", "distance_m": d})
        prev = cur
    steps.append({"instruction": "Arrive", "distance_m": 0})
    return steps


# --- FastAPI service ----------------------------------------------------------
class RouteRequest(BaseModel):
    start: str
    goal: str
    context: Optional[Dict[str, Any]] = None


class RouteCoordsRequest(BaseModel):
    start_lat: float
    start_lon: float
    goal_lat: float
    goal_lon: float
    context: Optional[Dict[str, Any]] = None



# --- ML Integration -----------------------------------------------------------
class ETAEstimator:
    """
    ML-based ETA Estimator. Loads `eta_model.joblib` if available.
    """
    def __init__(self, model_path: str = "eta_model.joblib"):
        self.model = None
        if os.path.exists(model_path):
            try:
                self.model = joblib.load(model_path)
                print(f"Loaded ETA model from {model_path}")
            except Exception as e:
                print(f"Failed to load ETA model: {e}")
        else:
            print(f"No ETA model found at {model_path}, using fallback.")

    def predict_batch(self, edges: List[Edge], hour: int, context: Optional[Dict[str, Any]] = None) -> List[float]:
        """
        Batch prediction for multiple edges. Optimized for A* pre-computation.
        """
        # 1. Prepare Features
        weather_val = 0
        if context and context.get("weather") == "rain": weather_val = 1
        
        incident_val = 0
        if context and context.get("incident"): incident_val = context["incident"]
        
        # If no model, use fallback logic for all
        if self.model is None:
            results = []
            for edge in edges:
                base = edge.live_eta()
                if 7 <= hour <= 9 or 16 <= hour <= 18: base *= 1.15
                if weather_val == 1: base *= 1.1
                results.append(base)
            return results

        # 2. Build Feature Matrix
        features = []
        for edge in edges:
            features.append([
                edge.length_m,
                edge.typical_speed_mps,
                hour,
                weather_val,
                incident_val
            ])
            
        # 3. Fast Vectorized Inference
        try:
            # Predict all at once
            preds = self.model.predict(features)
            return [max(p, 1.0) for p in preds]
        except Exception as e:
            print(f"Batch predict failed: {e}")
            # Fallback
            return [e.live_eta() for e in edges]

    def predict(self, edge: Edge, hour: int, context: Optional[Dict[str, Any]] = None) -> float:
        return self.predict_batch([edge], hour, context)[0]

class DemandEstimator:
    """
    Predicts emergency demand (risk) for regions.
    """
    def __init__(self, model_path: str = "demand_model.joblib"):
        self.model = None
        if os.path.exists(model_path):
            try:
                self.model = joblib.load(model_path)
                print(f"Loaded Demand model from {model_path}")
            except Exception as e:
                print(f"Failed to load Demand model: {e}")

    def predict_risk(self, region_id: int, hour: int, day_of_week: int, weather: int) -> float:
        if self.model is None:
            return 0.0 # Default low risk
        
        try:
            features = [[region_id, hour, day_of_week, weather]]
            # predict returns class (0 or 1), or we can use predict_proba for risk score
            # predict_proba returns [[prob_0, prob_1]]
            probs = self.model.predict_proba(features)[0]
            return probs[1] # Probability of High Risk
        except Exception:
            return 0.0


# --- Updated TrafficRouter using Estimators ---
class TrafficRouter:
    def __init__(self, graph: Graph, eta_estimator: Optional[ETAEstimator] = None):
        self.graph = graph
        self.eta_estimator = eta_estimator or ETAEstimator()
        self.demand_estimator = DemandEstimator() # Auto-load
        self.base_costs: Dict[int, float] = {}   # id(edge) -> nominal traversal time
        self.dynamic_penalties: Dict[int, float] = {} # id(edge) -> penalty multiplier
        self.last_context_hash = None
        self.global_factor = 1.0
        
        
        # Precompute base costs once
        self._rebuild_base_costs()

    def set_graph(self, new_graph: Graph):
        """Update graph and rebuild caches."""
        self.graph = new_graph
        self._rebuild_base_costs()

    def _rebuild_base_costs(self):
        """Internal helper to compute base costs."""
        self.base_costs.clear()
        for edges in self.graph.adjacency.values():
            for e in edges:
                 # Base time = length / typical_speed + turn_penalty
                 # We treat this as the invariant part
                 cost = e.length_m / max(e.typical_speed_mps, 0.1) + e.turn_penalty_s
                 self.base_costs[id(e)] = cost

    def _update_global_factor(self, context: Optional[Dict[str, Any]] = None):
        """Compute naive global factors (rain, time) once per request."""
        factor = 1.0
        if context:
             if context.get("weather") == "rain": factor *= 1.1
             # We could add time-of-day checks here, but let's keep it simple and fast
        self.global_factor = factor

    def _edge_cost(self, edge: Edge, context: Optional[Dict[str, Any]] = None, mode: str = "ai") -> float:
        if mode == "standard":
             # Standard GPS assumption
             return self.base_costs.get(id(edge), edge.length_m / max(edge.typical_speed_mps, 0.1))
             
        # AI Mode - Optimized
        # nominal * global_factor * specific_penalty
        
        # 1. Base Cost (Cached)
        cost = self.base_costs.get(id(edge), 0.0)
        
        # 2. Dynamic Penalty (Live Incidents)
        if id(edge) in self.dynamic_penalties:
             cost *= self.dynamic_penalties[id(edge)]
             
        # 3. Global Factor (Weather/Time) - updated separately
        cost *= self.global_factor
        
        return cost
    
    def update_live_feeds(
        self,
        live_speeds: Dict[Tuple[str, str], float],
        blocks: Set[Tuple[str, str]],
    ) -> None:
        """
        Updates internal penalty cache based on live data.
        """
        self.dynamic_penalties.clear()
        
        # Create a quick lookup for edges? 
        # Actually we can iterate graph or iterate inputs. 
        # Iterating graph is safer ensuring we find the edge objects.
        # But O(Edges) is better than O(V) if graph is large? No, graph is small-ish.
        # Let's iterate inputs if possible, but we need the Edge objects.
        # Map (u,v) -> Edge would be faster.
        
        # Build temp index if not exists (or we could maintain it)
        # For now, let's just do the safe iteration we had, but optimize what we store.
        
        # Optimization: Build a (u,v)->edge map once? 
        # Doing it on the fly:
        
        for src, edges in self.graph.adjacency.items():
            for e in edges:
                key = (src, e.target)
                
                # Check Blockage
                if key in blocks:
                    e.blocked = True
                    # Massive penalty for A* to avoid (soft block) or hard block
                    self.dynamic_penalties[id(e)] = 1000.0 
                else:
                    e.blocked = False # Autoclear if not in blocks (simplification)
                
                # Check Speed
                if key in live_speeds:
                    live_s = live_speeds[key]
                    ratio = e.typical_speed_mps / max(live_s, 0.1)
                    if ratio > 1.0:
                         self.dynamic_penalties[id(e)] = ratio


    def route(self, start: str, goal: str, context: Optional[Dict[str, Any]] = None, mode: str = "ai") -> Tuple[float, List[str], int]:
        heuristic_fn = lambda node: fast_heuristic(self.graph, node, goal)
        return astar(
            self.graph,
            start,
            goal,
            cost_fn=lambda e: self._edge_cost(e, context, mode=mode),
            heuristic_fn=heuristic_fn,
        )

# --- Graph Loading Utilities ---
CACHE_DIR = Path("graph_cache")
CACHE_DIR.mkdir(exist_ok=True)

def download_graph_with_timeout(lat: float, lon: float, radius: int, timeout: int = 30) -> Optional[Graph]:
    """Downloads graph, effectively ignoring timeout for simplicty in this fix."""
    try:
        return load_graph_from_osm(point=(lat, lon), dist=radius)
    except Exception as e:
        print(f"Download failed: {e}", flush=True)
        return None

def save_graph_cache(graph: Graph, path: Path):
    joblib.dump(graph, path)

def load_graph_cache(path: Path) -> Graph:
    return joblib.load(path)



# --- Global State for Render/Uvicorn ---
graph = Graph()
router: Optional[TrafficRouter] = None
eta_estimator: Optional[ETAEstimator] = None

# --- FastAPI service updates ---
def build_app() -> Any:

    """
    Build a FastAPI app serving /route. Requires fastapi+pydantic+uvicorn installed.
    """
    if FastAPI is None:
        raise ImportError("fastapi and pydantic are required. Install with `pip install fastapi uvicorn`.")
    
    global router, eta_estimator
    
    # Use ML estimator if not provided
    if eta_estimator is None:
        eta_estimator = ETAEstimator()
        
    router = TrafficRouter(graph, eta_estimator)
    app = FastAPI()
    
    from fastapi.middleware.cors import CORSMiddleware
    app.add_middleware(
        CORSMiddleware,
        allow_origins=["*"],
        allow_credentials=True,
        allow_methods=["*"],
        allow_headers=["*"],
    )

    @app.on_event("startup")
    async def startup_event():
        global graph, router, loaded_bbox
        print("Executing startup... checking for graph cache.", flush=True)
        try:
            cache_path = Path("graph_cache/graph.pkl")
            if cache_path.exists():
                print(f"Loading graph from {cache_path}...", flush=True)
                loaded_graph = joblib.load(cache_path)
                if loaded_graph:
                    graph = loaded_graph
                    if graph._kdtree is None:
                        graph.build_kdtree() # Ensure tree is built
                    # router.graph = loaded_graph <--- OLD BUGGY WAY
                    router.set_graph(loaded_graph) # Correctly rebuilds costs
                    print(f"Graph loaded from cache. Nodes: {len(graph.coordinates)}", flush=True)
            else:
                print("No graph cache found.", flush=True)
        except Exception as e:
            print(f"Failed to load cache on startup: {e}", flush=True)

    # Loaded BBox State
    loaded_bbox: Optional[Tuple[float, float, float, float]] = None

    # Global caching for incidents
    _incident_cache = {
        "data": [],
        "timestamp": 0.0
    }
    
    def process_incidents_to_penalties(target_graph: Graph, incidents: List[Dict[str, Any]]) -> Tuple[Dict, Set]:
        """Helper to convert list of incidents to router penalties."""
        live_speeds = {}
        blocks = set()
        
        for inc in incidents:
              try:
                   n = nearest_node(target_graph, inc['lat'], inc['lon'])
                   for e in target_graph.neighbors(n):
                        severity = inc.get('severity', 'Medium')
                        
                        if "closure" in inc.get("type", "") or "closed" in inc.get("subtype", "").lower():
                             blocks.add((n, e.target))
                        else:
                             # Slow down - TUNED: High=40% speed (was 20%), Med=70% speed (was 50%)
                             factor = 0.4 if severity == "High" else 0.7
                             live_speeds[(n, e.target)] = e.typical_speed_mps * factor
              except Exception:
                   continue
        return live_speeds, blocks

    def update_traffic_state():
        """Refreshes incidents and updates global router penalties."""
        global graph, router
        if graph is None or router is None: return
        
        now = time.time()
        # Refresh cache if older than 10 seconds (check stability elsewhere)
        # But for simulated data, we want stability based on time windows.
        # If cache is populated and valid window, skip.
        
        # 30 second refresh check -> Increased to 300s (5 mins) for UI stability
        if now - _incident_cache["timestamp"] < 300 and _incident_cache["data"]:
             # print("DEBUG: Using cached traffic state.", flush=True)
             return

        # 1. Fetch Real or Generate Simulated
        active_incidents = []
        source_type = "Simulated"
        
        if fetcher:
            # Use real data
             try:
                 bbox = (14.35, 121.00, 14.45, 121.10) 
                 found = fetcher.fetch_incidents(bbox)
                 if found:
                      active_incidents = found
                      source_type = "Real (TomTom)"
             except Exception as e:
                 print(f"Fetch error: {e}")
        
        if not active_incidents:
            # Simulated fallback (Stable)
            source_type = "Simulated (Fixed Seed)"
            blackspots = [
                {"name": "Alabang Viaduct (Merge Point)", "lat": 14.4206, "lon": 121.0458, "risk": "Extreme"},
                {"name": "National Rd / Putatan H-way", "lat": 14.3955, "lon": 121.0435, "risk": "High"},
                {"name": "Sucat Interchange", "lat": 14.4447, "lon": 121.0475, "risk": "High"},
                {"name": "Tunasan Sharp Curve", "lat": 14.3768, "lon": 121.0505, "risk": "Medium"},
                {"name": "Filinvest Ave Intersection", "lat": 14.4140, "lon": 121.0420, "risk": "Medium"}
            ]
            
            import random
            seed_val = int(time.time() / 300) # 5 min fixed window
            rng = random.Random(seed_val)
            
            for spot in blackspots:
                if rng.random() < 0.4:
                    lat_jit = spot["lat"] + (rng.random() - 0.5) * 0.002
                    lon_jit = spot["lon"] + (rng.random() - 0.5) * 0.002
                    type_event = "accident" if rng.random() < 0.7 else "closure"
                    subtype = "Car Crash"
                    if type_event == "closure": subtype = "Road Construction"
                    
                    active_incidents.append({
                        "lat": lat_jit,
                        "lon": lon_jit,
                        "type": type_event,
                        "subtype": subtype,
                        "severity": spot["risk"],
                        "description": f"Incident at {spot['name']}"
                    })

        # 2. Update Cache
        _incident_cache["data"] = active_incidents
        _incident_cache["timestamp"] = now
        
        # 3. Sync Router (Global)
        live_speeds, blocks = process_incidents_to_penalties(graph, active_incidents)
        router.update_live_feeds(live_speeds, blocks)
        print(f"DEBUG: Traffic state updated ({source_type}). {len(active_incidents)} incidents. {len(blocks)} blocks.", flush=True)





    @app.post("/route")
    def route(req: RouteRequest) -> Dict[str, Any]:
        try:
            eta, path = router.route(req.start, req.goal, context=req.context)
            pcs = path_to_coords(graph, path)
            return {
                "eta_seconds": eta,
                "path": path,
                "path_coords": pcs,
                "steps": steps_from_coords(pcs),
                "status": "ok",
            }
        except ValueError as e:
            return {"status": "no_path", "error": str(e)}

    @app.post("/route/coords")
    def route_by_coords(req: RouteCoordsRequest) -> Dict[str, Any]:
        try:
            global graph, router
            print(f"--- Route Request: {req.start_lat},{req.start_lon} -> {req.goal_lat},{req.goal_lon}", flush=True)
            if graph is None: print("ERROR: graph is None", flush=True)
            else: print(f"DEBUG: graph nodes: {len(graph.adjacency)}", flush=True)

            # 1. Loading / Caching Optimization
            # Try to reuse the existing global graph first to avoid slow downloads
            if graph is not None and len(graph.adjacency) > 0:
                # Check if points are roughly within existing graph bounds (heuristic)
                # Or just assume for this demo that we stay in Muntinlupa
                 local_graph = graph
                 print("Reusing existing global graph (Fast Mode)", flush=True)
            else:
                 # Fallback to dynamic loading (Slow)
                 print("Graph not loaded. Downloading...", flush=True)
                 center_lat = (req.start_lat + req.goal_lat) / 2
                 center_lon = (req.start_lon + req.goal_lon) / 2
                 import math
                 dlat = req.goal_lat - req.start_lat
                 dlon = req.goal_lon - req.start_lon
                 route_dist_deg = math.sqrt(dlat**2 + dlon**2)
                 # radius_m = max(3000, int((route_dist_deg * 111000 / 2) + 2000))
                 radius_m = 1000 # Force small radius to avoid timeout
                 
                 local_graph = download_graph_with_timeout(center_lat, center_lon, radius_m, timeout=60)
                 if local_graph is None:
                      raise ValueError("Could not download map from OSM and no cache available.")
                 
                 # Update global
                 graph = local_graph
                 router.set_graph(local_graph) # Rebuild cache for new graph


            loc_router = TrafficRouter(local_graph, eta_estimator)
            
            try:
                start_node = nearest_node(local_graph, req.start_lat, req.start_lon)
                goal_node = nearest_node(local_graph, req.goal_lat, req.goal_lon)
            except Exception:
                return {"status": "no_path", "error": "Cannot snap to node"}

            # ----------------------------------------------------
            # 2. Fetch Live Traffic (Moved BEFORE routing)
            # ----------------------------------------------------
            import traceback
            traffic_edges = []
            
            # ----------------------------------------------------
            # 2. Fetch Live Traffic (Unified)
            # ----------------------------------------------------
            # Ensure global state is fresh
            update_traffic_state()
            
            # Use the GLOBAL router if we are using the GLOBAL graph
            # This ensures we share the penalties we just computed in update_traffic_state
            if local_graph is graph:
                 print("Using global router instance", flush=True)
                 active_router = router
            else:
                 # We have a specific local graph (downloaded freshly)
                 # We must manually sync the penalties from the global state or re-compute
                 print("Using local router instance (syncing)", flush=True)
                 active_router = loc_router
                 # Sync penalties that apply to this subgraph
                 # It's faster to just re-apply the incidents to this new small graph
                 cached_incidents = _incident_cache["data"]
                 live_speeds, blocks = process_incidents_to_penalties(local_graph, cached_incidents)
                 active_router.update_live_feeds(live_speeds, blocks)
            
            active_router._update_global_factor(req.context)
            
            # Capture traffic edges for response (Visuals)
            traffic_edges = []
            # We can reconstruct them from the incident cache for the response
            # Or just ignore, since frontend calls /incidents separately
            
             # ----------------------------------------------------
            # 3. Calculate Routes
            # ----------------------------------------------------
            # 1. AI Route (Optimized)
            try:
                eta_ai, path_ai, visited_ai = active_router.route(start_node, goal_node, context=req.context, mode="ai")
            except ValueError:
                 return {"status": "no_path", "error": "No AI path found"}

            # 2. Standard Route (Baseline)
            try:
                eta_std_planned, path_std, _ = active_router.route(start_node, goal_node, context=req.context, mode="standard")
            except ValueError:
                path_std = path_ai
                
            # 3. Calculate REAL-WORLD ETA for Standard Route
            eta_std_real = 0.0
            if path_std:
                 for i in range(len(path_std) - 1):
                      u, v = path_std[i], path_std[i+1]
                      # Find edge
                      target_edge = None
                      for e in local_graph.neighbors(u):
                           if e.target == v:
                                target_edge = e
                                break
                      if target_edge:
                           # Cost using AI logic (Real world penalties)
                           eta_std_real += active_router._edge_cost(target_edge, context=req.context, mode="ai")
            
            time_saved = eta_std_real - eta_ai
            
            # Diagnostic Stats
            len_ai = sum(local_graph.adjacency[path_ai[i]][0].length_m for i in range(len(path_ai)-1)) if len(path_ai) > 1 else 0
            len_std = sum(local_graph.adjacency[path_std[i]][0].length_m for i in range(len(path_std)-1)) if path_std and len(path_std) > 1 else 0
            
            avg_speed_ai = (len_ai / eta_ai) * 3.6 if eta_ai > 0 else 0
            avg_speed_std = (len_std / eta_std_real) * 3.6 if eta_std_real > 0 else 0
            
            print(f"--- Route Comparison ---", flush=True)
            print(f"Standard Route: {len(path_std)} nodes, {len_std/1000:.2f} km, {eta_std_real:.1f}s ~ {int(avg_speed_std)} km/h")
            print(f"AI Route:       {len(path_ai)} nodes, {len_ai/1000:.2f} km, {eta_ai:.1f}s ~ {int(avg_speed_ai)} km/h")
            print(f"Time Saved: {time_saved:.1f}s")
            
            # Check for ghost penalties
            if abs(eta_std_real - eta_ai) > 1.0 and len(blocks) == 0 and len(live_speeds) == 0:
                 print("WARNING: Divergence detected despite NO active incidents via feeds.")
                 # Print first few edge costs to see why
                 if path_std and len(path_std) > 1:
                      u, v = path_std[0], path_std[1]
                      for e in local_graph.neighbors(u):
                           if e.target == v:
                                base = active_router.base_costs.get(id(e), 0)
                                dyn = active_router.dynamic_penalties.get(id(e), 1.0)
                                print(f"DEBUG Standard Edge 0: {u}->{v} Length: {e.length_m}m Speed: {e.typical_speed_mps}mps BaseCost: {base:.1f} Penalty: {dyn}")
                                

            if time_saved > 0:
                 print("AI Recommendation: DETOUR (Faster)", flush=True)
            else:
                 print("AI Recommendation: STANDARD (Optimal)", flush=True)
            print("------------------------", flush=True)

            coords_ai = path_to_coords(local_graph, path_ai)
            coords_std = path_to_coords(local_graph, path_std)
            
            steps = steps_from_coords(coords_ai) if coords_ai else []
            
            return {
                "status": "ok",
                
                # AI Data
                "eta_seconds": eta_ai,
                "path": path_ai,
                "path_coords": coords_ai,
                
                # Comparison Data
                "eta_std_real": eta_std_real,
                "path_std_coords": coords_std,
                "time_saved_seconds": max(0, time_saved),
                
                "steps": steps,
                "traffic": traffic_edges,
                "visited_nodes": visited_ai,
                "graph_source": local_graph.source
            }

        except Exception as e:
             import traceback
             with open("crash.log", "w") as f:
                 f.write(f"Crash detected: {e}\n")
                 traceback.print_exc(file=f)
             traceback.print_exc()
             return {"status": "error", "error": str(e)}


    def predict_demand(hour: Optional[int] = None) -> Dict[str, Any]:
        """
        Returns predicted risk for all regions.
        """
        if hour is None:
            hour = time.localtime().tm_hour
        day = time.localtime().tm_wday
        weather = 0 # Assume clear for demo
        
        risks = {}
        # Iterate regions 0-4
        for r_id in range(5):
            prob = router.demand_estimator.predict_risk(r_id, hour, day, weather)
            risks[f"region_{r_id}"] = prob
            
        return {
            "status": "ok",
            "hour": hour,
            "predictions": risks,
            "message": "Higher value means higher probability of emergency."
        }



    @app.get("/config")
    def config() -> Dict[str, Any]:
        token = os.environ.get("MAPBOX_TOKEN")
        inc_url = os.environ.get("INCIDENTS_URL")
        # Expose TomTom key for frontend map tiles
        tt_key = TOMTOM_API_KEY 
        return {
            "status": "ok", 
            "mapbox_token": token, 
            "incidents_url": inc_url,
            "tomtom_key": tt_key
        }

    @app.get("/incidents")
    def incidents_feed() -> Dict[str, Any]:
        """
        Returns accidents. Sources from the shared cache.
        """
        update_traffic_state()
        return {"status": "ok", "incidents": _incident_cache["data"]}


    @app.get("/predict/demand")
    def predict_demand(hour: Optional[int] = None) -> Dict[str, Any]:
        """
        Returns predicted risk for all regions.
        """
        if hour is None:
            hour = time.localtime().tm_hour
        day = time.localtime().tm_wday
        weather = 0 # Assume clear for demo
        
        risks = {}
        # Iterate regions 0-9 (Frontend uses 0-9)
        for r_id in range(10):
            prob = router.demand_estimator.predict_risk(r_id, hour, day, weather)
            risks[f"region_{r_id}"] = prob
            
        return {
            "status": "ok",
            "hour": hour,
            "predictions": risks,
            "message": "Higher value means higher probability of emergency."
        }

    @app.get("/traffic")
    def traffic_status() -> Dict[str, Any]:
        """
        Return all edges with their congestion status.
        Now synced with the ACTUAL router penalties.
        """
        update_traffic_state() # Ensure fresh
        
        edges_data = []
        for u, neighbors in graph.adjacency.items():
            if u not in graph.coordinates: continue
            u_coords = graph.coordinates[u]
            
            for e in neighbors:
                if e.target not in graph.coordinates: continue
                v_coords = graph.coordinates[e.target]
                
                # Check Router State for this edge
                penalty = router.dynamic_penalties.get(id(e), 1.0)
                
                status = "free"
                congestion = 0.0
                
                if penalty >= 1000.0: # Blocked
                    status = "heavy" # or blocked
                    congestion = 1.0
                elif penalty > 2.0:
                    status = "heavy"
                    congestion = 0.8
                elif penalty > 1.2:
                    status = "moderate"
                    congestion = 0.4
                
                if status != "free":
                    geom_pts = []
                    if e.geometry:
                         geom_pts = [{"lat": p[0], "lon": p[1]} for p in e.geometry]
                    edges_data.append({
                        "u": {"lat": u_coords[0], "lon": u_coords[1]},
                        "v": {"lat": v_coords[0], "lon": v_coords[1]},
                        "status": status,
                        "congestion": congestion,
                        "geometry": geom_pts
                    })
        
        # print(f"DEBUG: Returning {len(edges_data)} traffic segments.", flush=True)
        return {"status": "ok", "edges": edges_data}

    @app.post("/graph/load_bbox")
    def load_bbox(req: Dict[str, float]) -> Dict[str, Any]:
        global graph, router, loaded_bbox
        try:
            n = float(req.get("north"))
            s = float(req.get("south"))
            e = float(req.get("east"))
            w = float(req.get("west"))
        except Exception:
            return {"status": "error", "error": "invalid bbox"}
            
        # FIX: If we already have a detailed graph (from cache), don't overwrite it with a failed download/grid
        if graph and len(graph.coordinates) > 1000:
             print("DEBUG: detailed graph already loaded, skipping load_bbox overwrite.", flush=True)
             return {"status": "ok", "nodes": len(graph.coordinates), "bbox": {"north": n, "south": s, "east": e, "west": w}, "source": "cache_preserved"}

        try:
            if ox is None:
                raise ImportError("osmnx missing")
                # new_graph = build_sample_graph()
                # source = "sample"
            else:
                new_graph = load_graph_from_osm(north=n, south=s, east=e, west=w)
                source = "osm"
        except Exception as exc:
            # new_graph = build_sample_graph() # No fallback to fake data
            # source = "sample_fallback"
            print(f"Failed to load graph: {exc}")
            raise exc
        graph = new_graph
        # router.graph = new_graph <--- OLD BUGGY WAY
        router.set_graph(new_graph)
        loaded_bbox = (n, s, e, w)
        return {"status": "ok", "nodes": len(new_graph.coordinates), "bbox": {"north": n, "south": s, "east": e, "west": w}, "source": source}


    # Mount static files at root (Last to allow API routes to take precedence)
    static_dir = Path(__file__).parent / "docs"
    if static_dir.exists():
        app.mount("/", StaticFiles(directory=static_dir, html=True), name="static")

    return app


# --- Caching & Helpers --------------------------------------------------------
GRAPH_CACHE_FILE = "city_graph.pkl"

def compute_graph_bounds(graph: Graph) -> Tuple[float, float, float, float]:
    """Return (north, south, east, west) of the graph."""
    if not graph.coordinates:
        return (0, 0, 0, 0)
    lats = [c[0] for c in graph.coordinates.values()]
    lons = [c[1] for c in graph.coordinates.values()]
    return (max(lats), min(lats), max(lons), min(lons))

def save_graph_cache(graph: Graph, filepath: str = GRAPH_CACHE_FILE) -> None:
    print(f"Saving graph to cache: {filepath}...")
    try:
        joblib.dump(graph, filepath)
        print("Graph saved.")
    except Exception as e:
        print(f"Failed to save graph cache: {e}")

def load_graph_cache(filepath: str = GRAPH_CACHE_FILE) -> Optional[Graph]:
    if os.path.exists(filepath):
        print(f"Loading graph from cache: {filepath}...")
        try:
            return joblib.load(filepath)
        except Exception as e:
            print(f"Failed to load cache: {e}")
    return None



# Expose 'app' object at module level for ASGIs (Render/Uvicorn)
app = build_app()

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Traffic-aware emergency router")
    sub = parser.add_subparsers(dest="command")



    serve_p = sub.add_parser("serve", help="run FastAPI server with OSM graph")
    serve_p.add_argument("--place", type=str, help="place name for OSM (optional, overrides lat/lon)")
    serve_p.add_argument("--lat", type=float, default=14.4168, help="center latitude (Alabang)")
    serve_p.add_argument("--lon", type=float, default=121.0422, help="center longitude (Alabang)")
    serve_p.add_argument("--dist", type=int, default=3000, help="radius in meters (3km covers immediate area)")
    serve_p.add_argument("--north", type=float, help="bounding box north")
    serve_p.add_argument("--south", type=float, help="bounding box south")
    serve_p.add_argument("--east", type=float, help="bounding box east")
    serve_p.add_argument("--west", type=float, help="bounding box west")
    serve_p.add_argument("--host", type=str, default="0.0.0.0", help="API bind host")
    serve_p.add_argument("--port", type=int, default=int(os.environ.get("PORT", 8000)), help="API bind port")
    serve_p.add_argument("--no-cache", action="store_true", help="Duplicate graph download")

    args = parser.parse_args()
    command = args.command or "serve"
    if not hasattr(args, "iterations"):
        args.iterations = 5

    if command == "serve":
        if FastAPI is None:
            raise ImportError("fastapi and uvicorn are required. Install with `pip install fastapi uvicorn`.")
        
        print("Starting server with ON-DEMAND graph loading...")
        print("Maps will be downloaded automatically when you request a route.")
        
        # Start with minimal sample graph (will be replaced on-demand)
        if os.path.exists("graph_cache/graph.pkl"):
             print("Loading default graph from cache...")
             try:
                 graph = joblib.load("graph_cache/graph.pkl")
             except:
                 graph = Graph() # Start empty, load on demand
             graph = Graph()
        
        # app is already built globally, but we might want to re-inject graph if we want?
        # Actually, global app uses global graph.
        # So we just update the global graph.
        # But wait, build_app was called at module level with empty graph.
        # If we load a graph here (in main), we must update the globals.
        
        # Update global graph reference
        import traffic_router
        traffic_router.graph = graph
        # And update router
        # But router was init with empty graph.
        # traffic_router.router.set_graph(graph)
        if traffic_router.router:
             traffic_router.router.set_graph(graph)
        
        # app = build_app(graph) # No longer needed, app is global

        try:
            import uvicorn  # type: ignore
        except ImportError as exc:
            raise ImportError("uvicorn is required to run the server. Install with `pip install uvicorn`.") from exc
        
        print(f"Server ready at http://{args.host}:{args.port}")
        uvicorn.run(app, host=args.host, port=args.port)
    else:
        parser.error(f"Unknown command {command}")
