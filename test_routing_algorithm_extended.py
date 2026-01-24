import pytest
import math
import sys
import os
import numpy as np
from typing import Dict, Any

# Ensure we can import from the current directory
sys.path.append(os.getcwd())

from traffic_router import Graph, TrafficRouter, Edge

# ----------------------------
# Fixture: Comprehensive Graph & Router
# ----------------------------
@pytest.fixture(scope="module")
def setup_comprehensive_router():
    """
    Creates a comprehensive synthetic graph for extensive testing.
    
    Structure:
          (A)----150m----(B)----200m----(C)
           |              |              |
         100m           100m           150m
           |              |              |
          (D)----200m----(E)----150m----(F)
           |              |              |
         150m           100m           100m
           |              |              |
          (G)----100m----(H)----200m----(I)
    
    Different speeds and conditions for comprehensive testing
    """
    graph = Graph()
    
    # Coordinates (Dummy Lat/Lon - Manila area simulation)
    graph.coordinates = {
        "A": (14.5995, 120.9842),  # Top-left
        "B": (14.5995, 120.9960),  # Top-center
        "C": (14.5995, 121.0078),  # Top-right
        "D": (14.5886, 120.9842),  # Mid-left
        "E": (14.5886, 120.9960),  # Mid-center
        "F": (14.5886, 121.0078),  # Mid-right
        "G": (14.5777, 120.9842),  # Bottom-left
        "H": (14.5777, 120.9960),  # Bottom-center
        "I": (14.5777, 121.0078),  # Bottom-right
        "J": (15.0000, 121.5000),  # Disconnected node
    }
    
    # Horizontal edges (Top row)
    graph.add_edge("A", "B", length_m=150.0, typical_speed_mps=15.0)  # Highway
    graph.add_edge("B", "C", length_m=200.0, typical_speed_mps=12.0)  # City road
    
    # Horizontal edges (Middle row)
    graph.add_edge("D", "E", length_m=200.0, typical_speed_mps=10.0)  # City road
    graph.add_edge("E", "F", length_m=150.0, typical_speed_mps=13.0)  # City road
    
    # Horizontal edges (Bottom row)
    graph.add_edge("G", "H", length_m=100.0, typical_speed_mps=20.0)  # Highway
    graph.add_edge("H", "I", length_m=200.0, typical_speed_mps=18.0)  # Highway
    
    # Vertical edges (Left column)
    graph.add_edge("A", "D", length_m=100.0, typical_speed_mps=12.0)  # City road
    graph.add_edge("D", "G", length_m=150.0, typical_speed_mps=14.0)  # City road
    
    # Vertical edges (Center column)
    graph.add_edge("B", "E", length_m=100.0, typical_speed_mps=11.0)  # City road
    graph.add_edge("E", "H", length_m=100.0, typical_speed_mps=16.0)  # Highway
    
    # Vertical edges (Right column)
    graph.add_edge("C", "F", length_m=150.0, typical_speed_mps=10.0)  # City road
    graph.add_edge("F", "I", length_m=100.0, typical_speed_mps=17.0)  # Highway
    
    # Add bidirectional edges for complete connectivity
    graph.add_edge("B", "A", length_m=150.0, typical_speed_mps=15.0)
    graph.add_edge("C", "B", length_m=200.0, typical_speed_mps=12.0)
    graph.add_edge("E", "D", length_m=200.0, typical_speed_mps=10.0)
    graph.add_edge("F", "E", length_m=150.0, typical_speed_mps=13.0)
    graph.add_edge("H", "G", length_m=100.0, typical_speed_mps=20.0)
    graph.add_edge("I", "H", length_m=200.0, typical_speed_mps=18.0)
    graph.add_edge("D", "A", length_m=100.0, typical_speed_mps=12.0)
    graph.add_edge("G", "D", length_m=150.0, typical_speed_mps=14.0)
    graph.add_edge("E", "B", length_m=100.0, typical_speed_mps=11.0)
    graph.add_edge("H", "E", length_m=100.0, typical_speed_mps=16.0)
    graph.add_edge("F", "C", length_m=150.0, typical_speed_mps=10.0)
    graph.add_edge("I", "F", length_m=100.0, typical_speed_mps=17.0)
    
    # Mock demand estimator (returns varying risk based on context)
    class AdvancedMockEstimator:
        def predict_risk(self, time_of_day=12, weather=0, road_type=0, traffic_vol=500):
            # Simple risk calculation
            risk = 0.0
            if weather == 1:  # Rain
                risk += 0.3
            if 7 <= time_of_day <= 9 or 17 <= time_of_day <= 19:  # Rush hour
                risk += 0.2
            if traffic_vol > 1000:
                risk += 0.3
            return min(risk, 1.0)
    
    router = TrafficRouter(graph, AdvancedMockEstimator())
    return router, graph

# ----------------------------
# TEST CASES (20 Total)
# ----------------------------

# GROUP 1: Standard Routing (No Traffic)
# ----------------------------
def test_TC01_shortest_path_horizontal(setup_comprehensive_router):
    """
    Test Case 1: Basic horizontal route A->B->C
    Scenario: Clear weather, normal speed
    Expected: Shortest time path
    """
    router, graph = setup_comprehensive_router
    router.dynamic_penalties = {}
    
    eta, path, _ = router.route("A", "C", context={})
    
    # A->B (150m/15mps=10s) + B->C (200m/12mps=16.67s) = ~26.67s
    assert "A" in path and "C" in path
    assert eta > 0

def test_TC02_shortest_path_vertical(setup_comprehensive_router):
    """
    Test Case 2: Basic vertical route A->D->G
    Scenario: Clear weather, normal speed
    Expected: Direct vertical path
    """
    router, graph = setup_comprehensive_router
    router.dynamic_penalties = {}
    
    eta, path, _ = router.route("A", "G", context={})
    
    # A->D (100m/12mps=8.33s) + D->G (150m/14mps=10.71s) = ~19.04s
    assert path == ["A", "D", "G"]
    assert math.isclose(eta, 19.05, abs_tol=1.0)

def test_TC03_diagonal_optimal_route(setup_comprehensive_router):
    """
    Test Case 3: Diagonal route A->I (top-left to bottom-right)
    Scenario: Find optimal path through grid
    Expected: Path through center or edges based on speed
    """
    router, graph = setup_comprehensive_router
    router.dynamic_penalties = {}
    
    eta, path, _ = router.route("A", "I", context={})
    
    assert path[0] == "A"
    assert path[-1] == "I"
    assert len(path) >= 5  # At least 4 edges to traverse

# GROUP 2: Traffic Congestion Scenarios
# ----------------------------
def test_TC04_detour_due_to_congestion(setup_comprehensive_router):
    """
    Test Case 4: Heavy traffic forces detour
    Scenario: Block A->B with heavy congestion (5x penalty)
    Expected: Router finds alternative path through D-E
    """
    router, graph = setup_comprehensive_router
    
    # Find and block A->B edge
    edge_ab = next(e for e in graph.neighbors("A") if e.target == "B")
    router.dynamic_penalties[id(edge_ab)] = 5.0
    
    eta, path, _ = router.route("A", "C", context={})
    
    # Should avoid A->B due to penalty
    # Possible alternative: A->D->E->... or other paths
    assert "A" in path and "C" in path

def test_TC05_multiple_congested_segments(setup_comprehensive_router):
    """
    Test Case 5: Multiple congested roads
    Scenario: Block A->B and D->E
    Expected: Router finds less congested alternative
    """
    router, graph = setup_comprehensive_router
    
    # Block multiple edges
    for source, target, penalty in [("A", "B", 4.0), ("D", "E", 4.0)]:
        edge = next((e for e in graph.neighbors(source) if e.target == target), None)
        if edge:
            router.dynamic_penalties[id(edge)] = penalty
    
    eta, path, _ = router.route("A", "F", context={})
    
    assert path[0] == "A"
    assert path[-1] == "F"
    # Path should avoid congested segments
    
    router.dynamic_penalties = {}

def test_TC06_moderate_traffic_adjustment(setup_comprehensive_router):
    """
    Test Case 6: Moderate traffic (2x penalty)
    Scenario: B->E has moderate congestion
    Expected: May still use route if overall time is acceptable
    """
    router, graph = setup_comprehensive_router
    
    edge_be = next((e for e in graph.neighbors("B") if e.target == "E"), None)
    if edge_be:
        router.dynamic_penalties[id(edge_be)] = 2.0
    
    eta, path, _ = router.route("B", "H", context={})
    
    assert path[0] == "B"
    assert path[-1] == "H"
    assert eta > 0
    
    router.dynamic_penalties = {}

# GROUP 3: Edge Cases
# ----------------------------
def test_TC07_same_start_and_destination(setup_comprehensive_router):
    """
    Test Case 7: Start equals destination
    Expected: Zero time, single node path
    """
    router, _ = setup_comprehensive_router
    
    eta, path, _ = router.route("E", "E", context={})
    
    assert eta == 0.0
    assert path == ["E"]

def test_TC08_disconnected_nodes(setup_comprehensive_router):
    """
    Test Case 8: Route to unreachable node
    Expected: ValueError with 'No path found' message
    """
    router, _ = setup_comprehensive_router
    
    with pytest.raises(ValueError, match="No path found"):
        router.route("A", "J", context={})

def test_TC09_reverse_route_symmetry(setup_comprehensive_router):
    """
    Test Case 9: Test route symmetry (A->D vs D->A)
    Expected: Similar times since bidirectional edges exist
    """
    router, graph = setup_comprehensive_router
    router.dynamic_penalties = {}
    
    eta_forward, path_forward, _ = router.route("A", "D", context={})
    eta_reverse, path_reverse, _ = router.route("D", "A", context={})
    
    # Times should be similar (within tolerance) for bidirectional edges
    assert math.isclose(eta_forward, eta_reverse, rel_tol=0.2)
    assert path_forward[0] == "A" and path_forward[-1] == "D"
    assert path_reverse[0] == "D" and path_reverse[-1] == "A"

# GROUP 4: Context-Aware Routing
# ----------------------------
def test_TC10_rush_hour_routing(setup_comprehensive_router):
    """
    Test Case 10: Rush hour context (7-9am)
    Scenario: Time of day affects route selection
    Expected: Route considers increased base traffic
    """
    router, graph = setup_comprehensive_router
    router.dynamic_penalties = {}
    
    context = {"time_of_day": 8, "weather": 0}  # 8am, clear
    eta, path, _ = router.route("A", "I", context=context)
    
    assert path[0] == "A"
    assert path[-1] == "I"
    assert eta > 0

def test_TC11_rainy_weather_routing(setup_comprehensive_router):
    """
    Test Case 11: Rainy weather conditions
    Scenario: Rain affects speed and risk
    Expected: Route may prefer highways over city roads
    """
    router, graph = setup_comprehensive_router
    
    context = {"time_of_day": 14, "weather": 1}  # 2pm, rain
    eta, path, _ = router.route("A", "I", context=context)
    
    assert path[0] == "A"
    assert path[-1] == "I"

def test_TC12_night_time_routing(setup_comprehensive_router):
    """
    Test Case 12: Night time routing (low traffic)
    Scenario: 3am with minimal traffic - Test with connected nodes
    Expected: Fastest theoretical route
    """
    router, graph = setup_comprehensive_router
    router.dynamic_penalties = {}
    
    context = {"time_of_day": 3, "weather": 0}
    eta, path, _ = router.route("A", "C", context=context)
    
    assert path[0] == "A"
    assert path[-1] == "C"
    assert eta > 0  # Should have valid ETA

# GROUP 5: Performance and Robustness
# ----------------------------
def test_TC13_maximum_penalties_scenario(setup_comprehensive_router):
    """
    Test Case 13: Extreme congestion on multiple routes
    Scenario: Apply maximum penalties (10x) to test limits
    Expected: Router still finds a valid path
    """
    router, graph = setup_comprehensive_router
    
    # Apply extreme penalties to several edges
    edge_ab = next((e for e in graph.neighbors("A") if e.target == "B"), None)
    edge_de = next((e for e in graph.neighbors("D") if e.target == "E"), None)
    
    if edge_ab:
        router.dynamic_penalties[id(edge_ab)] = 10.0
    if edge_de:
        router.dynamic_penalties[id(edge_de)] = 10.0
    
    eta, path, _ = router.route("A", "F", context={})
    
    assert path[0] == "A"
    assert path[-1] == "F"
    assert len(path) > 0
    
    router.dynamic_penalties = {}

def test_TC14_eta_calculation_precision(setup_comprehensive_router):
    """
    Test Case 14: Verify ETA calculation precision
    Scenario: Manual calculation vs router output
    Expected: Mathematical accuracy within tolerance
    """
    router, graph = setup_comprehensive_router
    router.dynamic_penalties = {}
    
    eta, path, _ = router.route("G", "H", context={})
    
    # G->H: 100m / 20mps = 5.0s
    assert path == ["G", "H"]
    assert math.isclose(eta, 5.0, abs_tol=0.1)

def test_TC15_alternative_path_comparison(setup_comprehensive_router):
    """
    Test Case 15: Compare two alternative routes
    Scenario: Force route through different paths and compare
    Expected: Router adapts ETA based on changing conditions
    """
    router, graph = setup_comprehensive_router
    router.dynamic_penalties = {}
    
    # Get baseline route A->C
    eta_baseline, path_baseline, _ = router.route("A", "C", context={})
    
    # Block direct path A->B, force alternative through D
    edge_ab = next((e for e in graph.neighbors("A") if e.target == "B"), None)
    edge_bc = next((e for e in graph.neighbors("B") if e.target == "C"), None)
    
    if edge_ab and edge_bc:
        router.dynamic_penalties[id(edge_ab)] = 15.0
        router.dynamic_penalties[id(edge_bc)] = 15.0
    
    eta_alt, path_alt, _ = router.route("A", "C", context={})
    
    # Alternative route should exist and have different characteristics
    assert path_alt[0] == "A" and path_alt[-1] == "C"
    # Either path is different OR ETA increased significantly due to penalties
    assert path_alt != path_baseline or eta_alt > eta_baseline * 2
    
    router.dynamic_penalties = {}

# GROUP 6: Advanced Scenarios
# ----------------------------
def test_TC16_cascading_traffic_impact(setup_comprehensive_router):
    """
    Test Case 16: Cascading traffic congestion
    Scenario: Congestion on B->E causes reroute affecting E->H
    Expected: Router adapts to multiple cascading changes
    """
    router, graph = setup_comprehensive_router
    
    # Simulate cascading congestion
    edges_to_congest = [("B", "E", 3.0), ("E", "H", 2.5)]
    for source, target, penalty in edges_to_congest:
        edge = next((e for e in graph.neighbors(source) if e.target == target), None)
        if edge:
            router.dynamic_penalties[id(edge)] = penalty
    
    eta, path, _ = router.route("B", "I", context={})
    
    assert path[0] == "B"
    assert path[-1] == "I"
    
    router.dynamic_penalties = {}

def test_TC17_highway_preference(setup_comprehensive_router):
    """
    Test Case 17: Highway vs city road preference
    Scenario: Choose between highway (faster) and city (shorter)
    Expected: Router prefers highway for time optimization
    """
    router, graph = setup_comprehensive_router
    router.dynamic_penalties = {}
    
    # G->H->I (highways) should be preferred for speed
    eta, path, _ = router.route("G", "I", context={})
    
    # Path through bottom row should be fast due to highway speeds
    assert path[0] == "G"
    assert path[-1] == "I"
    assert "H" in path  # Should go through highway route

def test_TC18_mixed_conditions_routing(setup_comprehensive_router):
    """
    Test Case 18: Mixed conditions (rain + rush hour + traffic)
    Scenario: Multiple adverse conditions simultaneously
    Expected: Router finds safest and reasonably fast path
    """
    router, graph = setup_comprehensive_router
    
    # Set penalties for rush hour simulation
    edge_be = next((e for e in graph.neighbors("B") if e.target == "E"), None)
    if edge_be:
        router.dynamic_penalties[id(edge_be)] = 2.5
    
    context = {"time_of_day": 8, "weather": 1, "traffic_vol": 1200}
    eta, path, _ = router.route("A", "I", context=context)
    
    assert path[0] == "A"
    assert path[-1] == "I"
    assert eta > 0
    
    router.dynamic_penalties = {}

def test_TC19_incremental_penalty_effect(setup_comprehensive_router):
    """
    Test Case 19: Test incremental penalty increases
    Scenario: Gradually increase penalty and observe route changes
    Expected: Route changes at threshold penalty values
    """
    router, graph = setup_comprehensive_router
    
    edge_ab = next((e for e in graph.neighbors("A") if e.target == "B"), None)
    
    # Test with low penalty
    if edge_ab:
        router.dynamic_penalties[id(edge_ab)] = 1.5
    eta_low, path_low, _ = router.route("A", "C", context={})
    
    # Test with high penalty
    if edge_ab:
        router.dynamic_penalties[id(edge_ab)] = 5.0
    eta_high, path_high, _ = router.route("A", "C", context={})
    
    # High penalty should result in longer ETA or different path
    assert eta_high != eta_low or path_high != path_low
    
    router.dynamic_penalties = {}

def test_TC20_comprehensive_stress_test(setup_comprehensive_router):
    """
    Test Case 20: Comprehensive stress test
    Scenario: Route through complex graph with multiple penalties
    Expected: System handles complex scenario without errors
    """
    router, graph = setup_comprehensive_router
    
    # Apply varied penalties across the graph
    penalties = [
        ("A", "B", 2.0),
        ("D", "E", 1.5),
        ("E", "F", 2.5),
        ("C", "F", 1.8),
    ]
    
    for source, target, penalty in penalties:
        edge = next((e for e in graph.neighbors(source) if e.target == target), None)
        if edge:
            router.dynamic_penalties[id(edge)] = penalty
    
    context = {"time_of_day": 17, "weather": 1, "traffic_vol": 1500}
    
    # Test multiple routes - use only connected paths
    routes_to_test = [("A", "I"), ("D", "F"), ("B", "H"), ("A", "F")]
    
    for start, end in routes_to_test:
        eta, path, _ = router.route(start, end, context=context)
        assert path[0] == start
        assert path[-1] == end
        assert eta > 0
    
    router.dynamic_penalties = {}

# ----------------------------
# Runner
# ----------------------------
if __name__ == "__main__":
    # Allow running directly with: python test_routing_algorithm_extended.py
    sys.exit(pytest.main(["-v", __file__]))
