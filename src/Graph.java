import java.util.HashMap;
import java.util.HashSet;
import java.util.Map;
import java.util.Set;

public class Graph {
  Map<Integer, Node> nodes;
  Map<Integer, Map<Integer, Double>> edges;

  public Graph() {
    nodes = new HashMap<>();
    edges = new HashMap<>();
  }

  public void addNode(int id, double x, double y) {
    nodes.put(id, new Node(id, x, y));
  }
  public boolean edgeExists(int node1, int node2) {
    return edges.containsKey(node1) && edges.get(node1).containsKey(node2);
  }
  public void addEdge(int node1, int node2, double weight) {
    edges.putIfAbsent(node1, new HashMap<>());
//    edges.putIfAbsent(node2, new HashMap<>());
    edges.get(node1).put(node2, weight);
//    edges.get(node2).put(node1, weight);
  }

  public double getEdgeWeight(int node1, int node2) {

    return edges.get(node1).get(node2);
  }

  public Map<Integer, Double> getSuccedors(int nodeId) {
//    System.out.println("getNeighbors: " + edges.get(nodeId));
    if(edges.get(nodeId) == null) {
      return new HashMap<>();
    }
    else {
      return edges.get(nodeId);
    }
  }

  public Set<Integer> getPredecessors(int nodeId) {
    Set<Integer> predecessors = new HashSet<>();
    for (int id : nodes.keySet()) {
      if (edges.containsKey(id) && edges.get(id).containsKey(nodeId)) {
        predecessors.add(id);
      }
    }
    return predecessors;
  }

  public Node getNode(int nodeId) {
    return nodes.get(nodeId);
  }
}
