"""
Scalability Testing for Emergency Routing Algorithm
Tests performance across different graph sizes: 9, 50, 200, 1000 nodes
"""

import time
import random
import statistics
from typing import List, Tuple
from traffic_router import Graph, TrafficRouter, astar, fast_heuristic

def generate_random_graph(num_nodes: int, avg_edges_per_node: int = 3) -> Graph:
    """
    Generate a random connected graph for scalability testing.
    
    Args:
        num_nodes: Number of nodes in the graph
        avg_edges_per_node: Average number of outgoing edges per node
    
    Returns:
        Graph object with random structure
    """
    graph = Graph()
    
    # Generate node coordinates in a grid-like pattern (simulating a city)
    # Use Manila-like coordinates (14.5-14.7 lat, 120.95-121.15 lon)
    for i in range(num_nodes):
        node_id = f"N{i}"
        # Distribute nodes in a square grid
        row = i // int(num_nodes ** 0.5)
        col = i % int(num_nodes ** 0.5)
        lat = 14.5 + (row * 0.2 / (num_nodes ** 0.5))
        lon = 120.95 + (col * 0.2 / (num_nodes ** 0.5))
        graph.coordinates[node_id] = (lat, lon)
    
    # Generate edges to ensure connectivity
    nodes = list(graph.coordinates.keys())
    
    # First, create a spanning tree to ensure all nodes are reachable
    for i in range(1, num_nodes):
        source = nodes[i]
        target = nodes[random.randint(0, i-1)]  # Connect to any previous node
        
        # Calculate distance
        lat1, lon1 = graph.coordinates[source]
        lat2, lon2 = graph.coordinates[target]
        dist_lat = abs(lat1 - lat2) * 111000  # approx meters
        dist_lon = abs(lon1 - lon2) * 111000
        length = (dist_lat**2 + dist_lon**2) ** 0.5
        
        # Random speed between 20-60 km/h (5.5-16.7 m/s)
        speed = random.uniform(5.5, 16.7)
        
        # Add bidirectional edges
        graph.add_edge(source, target, length, speed)
        graph.add_edge(target, source, length, speed)
    
    # Add additional random edges to increase connectivity
    total_edges = num_nodes * avg_edges_per_node // 2
    current_edges = num_nodes - 1
    
    attempts = 0
    max_attempts = total_edges * 5
    
    while current_edges < total_edges and attempts < max_attempts:
        source = random.choice(nodes)
        target = random.choice(nodes)
        
        if source == target:
            attempts += 1
            continue
        
        # Check if edge already exists
        existing = False
        for edge in graph.adjacency.get(source, []):
            if edge.target == target:
                existing = True
                break
        
        if existing:
            attempts += 1
            continue
        
        # Calculate distance
        lat1, lon1 = graph.coordinates[source]
        lat2, lon2 = graph.coordinates[target]
        dist_lat = abs(lat1 - lat2) * 111000
        dist_lon = abs(lon1 - lon2) * 111000
        length = (dist_lat**2 + dist_lon**2) ** 0.5
        
        speed = random.uniform(5.5, 16.7)
        
        # Add bidirectional edges
        graph.add_edge(source, target, length, speed)
        graph.add_edge(target, source, length, speed)
        
        current_edges += 1
        attempts += 1
    
    graph.source = f"Generated {num_nodes} nodes"
    return graph


def test_graph_routing(graph: Graph, num_tests: int = 20) -> List[float]:
    """
    Test routing on a graph with multiple random routes.
    
    Args:
        graph: The graph to test
        num_tests: Number of routing tests to perform
    
    Returns:
        List of execution times for each route
    """
    router = TrafficRouter(graph)
    nodes = list(graph.coordinates.keys())
    times = []
    
    for _ in range(num_tests):
        # Select random start and goal
        start = random.choice(nodes)
        goal = random.choice(nodes)
        
        # Ensure they're different
        while start == goal:
            goal = random.choice(nodes)
        
        # Time the routing
        start_time = time.perf_counter()
        try:
            cost, path, visited = router.route(start, goal)
            elapsed = time.perf_counter() - start_time
            times.append(elapsed)
        except ValueError:
            # Path not found (shouldn't happen with connected graph)
            pass
    
    return times


def run_scalability_tests():
    """
    Run comprehensive scalability tests across different graph sizes.
    """
    print("=" * 80)
    print("SCALABILITY TESTING - Emergency Routing Algorithm")
    print("=" * 80)
    print()
    
    # Test configurations: (num_nodes, avg_edges_per_node, label)
    test_configs = [
        (9, 3, "Small"),
        (50, 3, "Medium"),
        (200, 3, "Large"),
        (1000, 3, "Extra Large"),
    ]
    
    results = []
    
    for num_nodes, avg_edges, label in test_configs:
        print(f"\n{label} Graph ({num_nodes} nodes)")
        print("-" * 80)
        
        # Generate graph
        print(f"  Generating graph with ~{num_nodes * avg_edges // 2} edges...")
        graph = generate_random_graph(num_nodes, avg_edges)
        
        actual_edges = sum(len(edges) for edges in graph.adjacency.values())
        print(f"  Generated: {num_nodes} nodes, {actual_edges} directed edges ({actual_edges // 2} undirected)")
        
        # Test routing
        print(f"  Running 20 routing tests...")
        times = test_graph_routing(graph, num_tests=20)
        
        if times:
            avg_time = statistics.mean(times)
            std_dev = statistics.stdev(times) if len(times) > 1 else 0
            min_time = min(times)
            max_time = max(times)
            
            print(f"  Results:")
            print(f"    Average time: {avg_time:.4f} seconds")
            print(f"    Std deviation: {std_dev:.4f} seconds")
            print(f"    Min time: {min_time:.4f} seconds")
            print(f"    Max time: {max_time:.4f} seconds")
            
            results.append({
                'label': label,
                'nodes': num_nodes,
                'edges': actual_edges // 2,
                'avg_time': avg_time,
                'std_dev': std_dev,
                'min_time': min_time,
                'max_time': max_time,
            })
        else:
            print(f"  No successful routes found!")
    
    # Print summary table
    print("\n" + "=" * 80)
    print("SUMMARY TABLE")
    print("=" * 80)
    print()
    print(f"{'Graph Size':<15} {'Nodes':<8} {'Edges':<12} {'Avg Time/Route':<18} {'Std Dev':<12} {'Status'}")
    print("-" * 80)
    
    baseline_time = None
    for result in results:
        if baseline_time is None:
            baseline_time = result['avg_time']
            growth_rate = "Baseline"
        else:
            growth_factor = result['avg_time'] / baseline_time
            growth_rate = f"{growth_factor:.1f}×"
        
        print(f"{result['label']:<15} {result['nodes']:<8} {result['edges']:<12} "
              f"{result['avg_time']:.4f}s {' ':<8} {result['std_dev']:.4f}s {' ':<3} {growth_rate}")
    
    print()
    
    # Save results to file
    output_file = "scalability_test_results.txt"
    with open(output_file, 'w') as f:
        f.write("SCALABILITY TEST RESULTS\n")
        f.write("=" * 80 + "\n\n")
        
        f.write(f"{'Graph Size':<15} {'Nodes':<8} {'Edges':<12} {'Avg Time/Route':<18} {'Growth Rate'}\n")
        f.write("-" * 80 + "\n")
        
        baseline_time = None
        for result in results:
            if baseline_time is None:
                baseline_time = result['avg_time']
                growth_rate = "Baseline"
            else:
                growth_factor = result['avg_time'] / baseline_time
                growth_rate = f"{growth_factor:.1f}×"
            
            f.write(f"{result['label']:<15} {result['nodes']:<8} {result['edges']:<12} "
                   f"{result['avg_time']:.4f}s           {growth_rate}\n")
        
        f.write("\n\nDETAILED STATISTICS\n")
        f.write("=" * 80 + "\n\n")
        
        for result in results:
            f.write(f"{result['label']} Graph:\n")
            f.write(f"  Nodes: {result['nodes']}\n")
            f.write(f"  Edges: {result['edges']} (undirected)\n")
            f.write(f"  Average time: {result['avg_time']:.6f} seconds\n")
            f.write(f"  Std deviation: {result['std_dev']:.6f} seconds\n")
            f.write(f"  Min time: {result['min_time']:.6f} seconds\n")
            f.write(f"  Max time: {result['max_time']:.6f} seconds\n")
            f.write("\n")
    
    print(f"Results saved to: {output_file}")
    print()
    
    return results


if __name__ == "__main__":
    try:
        results = run_scalability_tests()
        print("\n✓ Scalability testing completed successfully!")
    except Exception as e:
        print(f"\n✗ Error during testing: {e}")
        import traceback
        traceback.print_exc()
