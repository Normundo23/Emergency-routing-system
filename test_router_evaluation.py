
import pytest
import math
import sys
import os
from typing import Dict, Any

# Ensure we can import from the current directory
sys.path.append(os.getcwd())

from traffic_router import Graph, TrafficRouter, Edge

# ----------------------------
# Fixture: Mock Graph & Router
# ----------------------------
@pytest.fixture(scope="module")
def setup_router():
    """
    Creates a simple synthetic graph for testing.
    Structure:
    
      (A) --100m--> (B) --100m--> (C)
       |             |
       +---200m------+
       
       A->B: 10m/s (10s)
       B->C: 10m/s (10s)
       A->C (direct): 10m/s (20s)
       
       So A->B->C = 20s
          A->C    = 20s
    """
    graph = Graph()
    
    # Coordinates (Dummy Lat/Lon)
    graph.coordinates = {
        "A": (14.0000, 121.0000),
        "B": (14.0010, 121.0000), # ~110m away
        "C": (14.0020, 121.0000), # ~110m away from B
        "D": (15.0000, 121.0000) # Disconnected
    }
    
    # Edges
    # A->B
    graph.add_edge("A", "B", length_m=100.0, typical_speed_mps=10.0)
    # B->C
    graph.add_edge("B", "C", length_m=100.0, typical_speed_mps=10.0)
    # A->C (Longer direct path)
    graph.add_edge("A", "C", length_m=250.0, typical_speed_mps=10.0)
    
    # Mock demand estimator (returns 0 risk)
    class MockEstimator:
        def predict_risk(self, *args): return 0.0
        
    router = TrafficRouter(graph, MockEstimator())
    return router, graph

# ----------------------------
# Correctness Tests
# ----------------------------
def test_TC01_standard_route_calculation(setup_router):
    """
    Verify that the router finds the shortest path by time in a clean env.
    Default: A->B->C is 20s. A->C is 25s.
    Should choose A->B->C.
    """
    router, graph = setup_router
    # Clear any penalties
    router.dynamic_penalties = {} 
    
    eta, path, _ = router.route("A", "C", context={})
    
    assert path == ["A", "B", "C"]
    assert math.isclose(eta, 20.0, abs_tol=0.1)

def test_TC02_ai_route_detour_logic(setup_router):
    """
    Verify that high congestion on the primary route causes a detour.
    Scenario: Block A->B.
    New optimal: A->C (25s).
    """
    router, graph = setup_router
    
    # Simulate heavy traffic on A->B
    # Find the edge ID for A->B
    edge_ab = next(e for e in graph.neighbors("A") if e.target == "B")
    
    # Apply high penalty (e.g., 5.0x cost)
    # Note: traffic_router uses id(edge) for dynamic_penalties
    router.dynamic_penalties[id(edge_ab)] = 5.0
    
    # Route again
    eta, path, _ = router.route("A", "C", context={})
    
    # Cost analysis:
    # A->B: 100m / 10mps = 10s * 5.0 penalty = 50s + B->C(10s) = 60s
    # A->C: 250m / 10mps = 25s * 1.0 penalty = 25s
    # Optimal should be A->C
    
    assert path == ["A", "C"]
    assert math.isclose(eta, 25.0, abs_tol=0.1)
    
    # Clean up
    router.dynamic_penalties = {}

def test_TC03_zero_length_path(setup_router):
    """
    Verify behavior when start == goal.
    """
    router, _ = setup_router
    eta, path, _ = router.route("A", "A", context={})
    assert eta == 0.0
    assert path == ["A"]

def test_TC04_disconnected_graph(setup_router):
    """
    Verify error handling for unreachable nodes.
    """
    router, _ = setup_router
    with pytest.raises(ValueError, match="No path found"):
        router.route("A", "D", context={})

# ----------------------------
# Dataset/Robustness Eval
# ----------------------------
def test_TC05_eta_calculation_accuracy(setup_router):
    """
    Mathematically verify the ETA cost function.
    """
    router, graph = setup_router
    router.base_costs = {} # Reset
    router.dynamic_penalties = {} # Reset
    
    edge = Edge(target="Z", length_m=1000, typical_speed_mps=20) # 50s
    
    # Register base cost manually since this edge isn't in the graph
    router.base_costs[id(edge)] = 50.0 # Length 1000 / Speed 20
    
    # Default cost
    cost_std = router._edge_cost(edge, {}, mode="standard")
    assert math.isclose(cost_std, 50.0)
    
    # AI cost with penalty
    router.dynamic_penalties[id(edge)] = 2.0
    cost_ai = router._edge_cost(edge, {}, mode="ai")
    
    # Expected: 50.0 * 2.0 * 1.0 (global) = 100.0
    assert math.isclose(cost_ai, 100.0)

if __name__ == "__main__":
    # Allow running directly
    sys.exit(pytest.main(["-v", __file__]))
