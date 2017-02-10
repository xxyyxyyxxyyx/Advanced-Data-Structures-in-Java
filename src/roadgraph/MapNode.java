package roadgraph;

import java.util.ArrayList;
import java.util.List;

import geography.GeographicPoint;

public class MapNode implements Comparable<MapNode>{
	/*
	 * Initializing class variables
	 * Variable location holds the geographic coordinates of the map node
	 * Variable eges holds the edges connected to the map node in an array
	 */
	private GeographicPoint location;
	private List<MapEdge> edges;
	private double length;
	
	// MapNode Constructor
	public MapNode(GeographicPoint point){
		location = point;
		edges = new ArrayList<MapEdge>();
		length = Double.MAX_VALUE;
	}
	// Returns the list of map npdes connected to the map node
	public List<MapNode> getNeighbours(){
		List<MapNode> neighbours = new ArrayList<>();
		for(MapEdge edge: edges){
			neighbours.add(edge.getOppositeNode(this));
		}
		return neighbours;
	}
	// Adds a map edge to the list of edges
	public void addEdge(MapEdge edge){
		edges.add(edge);
	}
	//returns the geographic coordinates of the map node
	public GeographicPoint getLocation(){
		return location;
	}
	public double getLength(){
		return length;
	}
	public List<MapEdge> getEdges(){
		return edges;
	}
	public void setLenght(double l){
		length = l;
	}
	@Override
	public int compareTo(MapNode b) {
		return Double.compare(this.getLength(), b.getLength());
	}
	
	public double estimatedLength(MapNode node){
		
		return this.getLocation().distance(node.getLocation());
		
	}
	public String toString(){
		return "("+location.x+","+location.y+")"+" "+length;
	}
	public List<MapNode> endNodes(){
		List<MapNode> output = new ArrayList<MapNode>();
		for(MapEdge edge : edges){
			if(!edge.getEndNode().equals(this)) output.add(edge.getEndNode());
		}
		return output;
	}
}
