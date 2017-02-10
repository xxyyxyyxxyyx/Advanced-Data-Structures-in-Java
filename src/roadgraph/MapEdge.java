package roadgraph;

public class MapEdge {
	// initializing class variables
	private MapNode start;
	private MapNode end;
	private String name;
	private String type;
	private double length;
	// MapEdge constructor
	public MapEdge(MapNode start,MapNode end,String name,String type, double length){
		this.start = start;
		this.end = end;
		this.name = name;
		this.type = type;
		this.length = length;
	}
	//return end node of the edge
	public MapNode getEndNode(){
		return end;
	}
	//returns start node of the edge
	public MapNode getStartNode(){
		return start;
	}
	//returns the neighbouring opposite node
	public MapNode getOppositeNode(MapNode node){
		if(node.equals(start)){
			return end;
		}
		else{
			return start;
		}
	}
	public MapNode getNeighbour(){
		if(this.equals(start)){
			return end;
		}
		else{
			return start;
		}
	}
	public double getLength(){
		return length;
	}
	public String toString(){
		return name;
	}
	
}
