/**
 * @author UCSD MOOC development team and YOU
 * 
 * A class which reprsents a graph of geographic locations
 * Nodes in the graph are intersections between 
 *
 */
package roadgraph;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
import java.util.HashSet;
import java.util.LinkedList;
import java.util.List;
import java.util.Map;
import java.util.PriorityQueue;
import java.util.Queue;
import java.util.Set;
import java.util.function.Consumer;

import geography.GeographicPoint;
import util.GraphLoader;

/**
 * @author UCSD MOOC development team and YOU
 * 
 *         A class which represents a graph of geographic locations Nodes in the
 *         graph are intersections between
 *
 */

public class MapGraph {

	private Map<GeographicPoint, MapNode> map;
	private int numOfEdges = 0;
	private int nodesVisited;
	// TODO: Add your member variables here in WEEK 3

	/**
	 * Create a new empty MapGraph
	 */
	public MapGraph() {
		// TODO: Implement in this constructor in WEEK 3
		map = new HashMap<GeographicPoint, MapNode>();
	}

	/**
	 * Get the number of vertices (road intersections) in the graph
	 * 
	 * @return The number of vertices in the graph.
	 */
	public int getNumVertices() {
		// TODO: Implement this method in WEEK 3
		return map.values().size();
	}

	public int getNodesVisited() {
		return nodesVisited;
	}

	/**
	 * Return the intersections, which are the vertices in this graph.
	 * 
	 * @return The vertices in this graph as GeographicPoints
	 */
	public Set<GeographicPoint> getVertices() {
		// TODO: Implement this method in WEEK 3
		return map.keySet();
	}

	/**
	 * Get the number of road segments in the graph
	 * 
	 * @return The number of edges in the graph.
	 */
	public int getNumEdges() {
		// TODO: Implement this method in WEEK 3
		return numOfEdges;
	}

	/**
	 * Add a node corresponding to an intersection at a Geographic Point If the
	 * location is already in the graph or null, this method does not change the
	 * graph.
	 * 
	 * @param location
	 *            The location of the intersection
	 * @return true if a node was added, false if it was not (the node was
	 *         already in the graph, or the parameter is null).
	 */
	public boolean addVertex(GeographicPoint location) {
		// TODO: Implement this method in WEEK 3
		if (map.containsKey(location) || location == null) {
			return false;
		}
		MapNode newNode = new MapNode(location);
		map.put(location, newNode);
		return true;
	}

	/**
	 * Adds a directed edge to the graph from pt1 to pt2. Precondition: Both
	 * GeographicPoints have already been added to the graph
	 * 
	 * @param from
	 *            The starting point of the edge
	 * @param to
	 *            The ending point of the edge
	 * @param roadName
	 *            The name of the road
	 * @param roadType
	 *            The type of the road
	 * @param length
	 *            The length of the road, in km
	 * @throws IllegalArgumentException
	 *             If the points have not already been added as nodes to the
	 *             graph, if any of the arguments is null, or if the length is
	 *             less than 0.
	 */
	public void addEdge(GeographicPoint from, GeographicPoint to, String roadName, String roadType, double length)
			throws IllegalArgumentException {

		// TODO: Implement this method in WEEK 3
		if (map.containsKey(from) && map.containsKey(to)) {
			MapNode start = map.get(from);
			MapNode end = map.get(to);

			MapEdge newEdge = new MapEdge(start, end, roadName, roadType, length);
			start.addEdge(newEdge);
			numOfEdges++;
		} else {
			throw new IllegalAccessError("The points have not been added");
		}

	}

	/**
	 * Find the path from start to goal using breadth first search
	 * 
	 * @param start
	 *            The starting location
	 * @param goal
	 *            The goal location
	 * @return The list of intersections that form the shortest (unweighted)
	 *         path from start to goal (including both start and goal).
	 */
	public List<GeographicPoint> bfs(GeographicPoint start, GeographicPoint goal) {
		// Dummy variable for calling the search algorithms
		Consumer<GeographicPoint> temp = (x) -> {
		};
		return bfs(start, goal, temp);
	}

	/**
	 * Find the path from start to goal using breadth first search
	 * 
	 * @param start
	 *            The starting location
	 * @param goal
	 *            The goal location
	 * @param nodeSearched
	 *            A hook for visualization. See assignment instructions for how
	 *            to use it.
	 * @return The list of intersections that form the shortest (unweighted)
	 *         path from start to goal (including both start and goal).
	 */
	public List<GeographicPoint> bfs(GeographicPoint start, GeographicPoint goal,
			Consumer<GeographicPoint> nodeSearched) {
		// TODO: Implement this method in WEEK 3

		// Hook for visualization. See writeup.
		// nodeSearched.accept(next.getLocation());
		Set<MapNode> visited = new HashSet<>();
		Queue<MapNode> toExplore = new LinkedList<>();
		Map<MapNode, MapNode> parentMap = new HashMap<MapNode, MapNode>();

		MapNode startNode = map.get(start);
		MapNode goalNode = map.get(goal);
		visited.add(startNode);
		toExplore.add(startNode);

		boolean found = false;
		while (!toExplore.isEmpty()) {
			MapNode currentNode = toExplore.remove();
			nodeSearched.accept(currentNode.getLocation());
			if (currentNode.equals(goalNode)) {
				found = true;
				break;
			}
			List<MapNode> neighbours = currentNode.getNeighbours();
			for (MapNode node : neighbours) {
				if (!visited.contains(node)) {
					visited.add(node);
					toExplore.add(node);
					parentMap.put(node, currentNode);
				}
			}
		}
		if (!found) {
			System.out.println("No path exists");
			return new ArrayList<GeographicPoint>();
		}

		// recosntructiong the path

		LinkedList<GeographicPoint> path = new LinkedList<GeographicPoint>();
		MapNode current = goalNode;
		while (!current.equals(startNode)) {
			path.addFirst(current.getLocation());
			current = parentMap.get(current);
		}
		path.addFirst(start);

		return path;
	}

	/**
	 * Find the path from start to goal using Dijkstra's algorithm
	 * 
	 * @param start
	 *            The starting location
	 * @param goal
	 *            The goal location
	 * @return The list of intersections that form the shortest path from start
	 *         to goal (including both start and goal).
	 */
	public List<GeographicPoint> dijkstra(GeographicPoint start, GeographicPoint goal) {
		// Dummy variable for calling the search algorithms
		// You do not need to change this method.
		Consumer<GeographicPoint> temp = (x) -> {
		};
		return dijkstra(start, goal, temp);
	}

	/**
	 * Find the path from start to goal using Dijkstra's algorithm
	 * 
	 * @param start
	 *            The starting location
	 * @param goal
	 *            The goal location
	 * @param nodeSearched
	 *            A hook for visualization. See assignment instructions for how
	 *            to use it.
	 * @return The list of intersections that form the shortest path from start
	 *         to goal (including both start and goal).
	 */
	public List<GeographicPoint> dijkstra(GeographicPoint start, GeographicPoint goal,
			Consumer<GeographicPoint> nodeSearched) {
		// TODO: Implement this method in WEEK 4

		// Hook for visualization. See writeup.
		// nodeSearched.accept(next.getLocation());
		nodesVisited = 0;
		PriorityQueue<MapNode> queue = new PriorityQueue<>();
		Set<MapNode> visited = new HashSet<>();
		Map<MapNode, MapNode> parentMap = new HashMap<MapNode, MapNode>();

		MapNode startNode = map.get(start);
		MapNode goalNode = map.get(goal);
		startNode.setLenght(0);
		queue.add(startNode);
		boolean found = false;
		while (!queue.isEmpty()) {
			MapNode curr = queue.remove();
			System.out.printf("DIjKSTRA visiting[NODE at location(Lat:%.2f Lon:%.2f) intersects streets: %s,] \n",
					curr.getLocation().getX(), curr.getLocation().getY(), curr.getEdges());
			nodesVisited++;
			nodeSearched.accept(curr.getLocation());
			if (!visited.contains(curr)) {
				visited.add(curr);
				if (curr == goalNode) {
					System.out.println("Nodes visited in search: " + nodesVisited);
					return route(startNode, goalNode, parentMap);
				}
				for (MapEdge edge : curr.getEdges()) {
					MapNode nextNode = edge.getEndNode();
					if (!visited.contains(nextNode)) {
						double lengthToNextNode = curr.getLength() + edge.getLength();
						if (lengthToNextNode < nextNode.getLength()) {
							nextNode.setLenght(lengthToNextNode);
							parentMap.put(nextNode, curr);
							queue.add(nextNode);
						}
					}
				}
			}
		}

		return new ArrayList<GeographicPoint>();

	}

	private List<GeographicPoint> route(MapNode startNode, MapNode goalNode, Map<MapNode, MapNode> parentMap) {
		LinkedList<GeographicPoint> route = new LinkedList<>();
		MapNode curr = goalNode;
		while (!curr.equals(startNode)) {
			route.addFirst(curr.getLocation());
			curr = parentMap.get(curr);
		}
		route.addFirst(startNode.getLocation());
		System.out.println("done");
		return route;
	}

	/**
	 * Find the path from start to goal using A-Star search
	 * 
	 * @param start
	 *            The starting location
	 * @param goal
	 *            The goal location
	 * @return The list of intersections that form the shortest path from start
	 *         to goal (including both start and goal).
	 */
	public List<GeographicPoint> aStarSearch(GeographicPoint start, GeographicPoint goal) {
		// Dummy variable for calling the search algorithms
		Consumer<GeographicPoint> temp = (x) -> {
		};
		return aStarSearch(start, goal, temp);
	}

	/**
	 * Find the path from start to goal using A-Star search
	 * 
	 * @param start
	 *            The starting location
	 * @param goal
	 *            The goal location
	 * @param nodeSearched
	 *            A hook for visualization. See assignment instructions for how
	 *            to use it.
	 * @return The list of intersections that form the shortest path from start
	 *         to goal (including both start and goal).
	 */
	public List<GeographicPoint> aStarSearch(GeographicPoint start, GeographicPoint goal,
			Consumer<GeographicPoint> nodeSearched) {
		// TODO: Implement this method in WEEK 4

		// Hook for visualization. See writeup.
		// nodeSearched.accept(next.getLocation());

		PriorityQueue<MapNode> queue = new PriorityQueue<>();
		HashSet<MapNode> visited = new HashSet<>();
		Map<MapNode, MapNode> parentMap = new HashMap<MapNode, MapNode>();

		nodesVisited = 0;

		MapNode startNode = map.get(start);
		MapNode goalNode = map.get(goal);

		startNode.setLenght(0);
		queue.add(startNode);

		while (!queue.isEmpty()) {
			MapNode curr = queue.remove();
			System.out.printf("A* visiting[NODE at location(Lat:%.2f Lon:%.2f) intersects streets: %s,] \n",
					curr.getLocation().getX(), curr.getLocation().getY(), curr.getEdges());
			nodesVisited++;
			nodeSearched.accept(curr.getLocation());

			if (!visited.contains(curr)) {

				visited.add(curr);
				if (curr.equals(goalNode)) {
					System.out.println("Nodes visited in search: " + nodesVisited);
					return route(startNode, goalNode, parentMap);

				}

				for (MapEdge edge : curr.getEdges()) {

					MapNode nextNode = edge.getEndNode();
					if (nextNode.equals(curr))
						break;
					if (!visited.contains(nextNode)) {

						double lengthToNextNode = curr.getLength()- curr.estimatedLength(goalNode) + edge.getLength();
						double estimatedLength = nextNode.estimatedLength(goalNode);
						double totalLength = lengthToNextNode + estimatedLength;

						if (nextNode.getLength() > totalLength) {
							nextNode.setLenght(totalLength);
							parentMap.put(nextNode, curr);
							queue.add(nextNode);
						}
					}

				}
			}

		}

		return new ArrayList<GeographicPoint>();
	}

	public static void main(String[] args) {

		MapGraph simpleTestMap = new MapGraph();
		GraphLoader.loadRoadMap("data/testdata/simpletest.map", simpleTestMap);

		GeographicPoint testStart = new GeographicPoint(1.0, 1.0);
		GeographicPoint testEnd = new GeographicPoint(8.0, -1.0);

//		System.out.println("Test 1 using simpletest: Dijkstra should be 9 and AStar should be 5");
		 List<GeographicPoint> testroute ; // = simpleTestMap.dijkstra(testStart,
		 // testEnd);
		List<GeographicPoint> testroute2; // = simpleTestMap.aStarSearch(testStart, testEnd);

		 MapGraph testMap = new MapGraph();
		 GraphLoader.loadRoadMap("data/maps/utc.map", testMap);
		
//		 // A very simple test using real data
//		 testStart = new GeographicPoint(32.869423, -117.220917);
//		 testEnd = new GeographicPoint(32.869255, -117.216927);
//		 System.out.println("Test 2 using utc: Dijkstra should be 13 and AStar should be 5");
//		 testroute = testMap.dijkstra(testStart, testEnd);
//		 testroute2 = testMap.aStarSearch(testStart, testEnd);
		
		 // A slightly more complex test using real data
		 testStart = new GeographicPoint(32.8674388, -117.2190213);
		 testEnd = new GeographicPoint(32.8697828, -117.2244506);
		 System.out.println("Test 3 using utc: Dijkstra should be 37 and AStar should be 10");
//		 testroute = testMap.dijkstra(testStart, testEnd);
		 testroute2 = testMap.aStarSearch(testStart, testEnd);

	}

}
