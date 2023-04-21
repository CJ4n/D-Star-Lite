import java.util.ArrayList;
import java.util.Arrays;
import java.util.Comparator;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.PriorityQueue;

public class DStarLite {

  Graph graph;
  int startId;
  int goalId;

  // Algorithm data structures
  PriorityQueue<NodeWrapper> U;
  Map<Integer, Double> gValues;
  Map<Integer, Double> rhsValues;
  Map<Integer, NodeWrapper> nodeWrapperMap;
  Comparator<NodeWrapper> nodeComparator;
  double Km = 0;

  public DStarLite(Graph graph, int startId, int goalId) {
    this.graph = graph;
    this.startId = startId;
    this.goalId = goalId;

    initialize();
  }

  private void initialize() {
    nodeComparator = (nodeId1, nodeId2) -> {
      var key1 = nodeId1.key;
      var key2 = nodeId2.key;
      if (key1[0] < key2[0]) {
        return -1;
      } else if (key1[0] > key2[0]) {
        return 1;
      } else {
        if (key1[1] < key2[1]) {
          return -1;
        } else if (key1[1] > key2[1]) {
          return 1;
        }
      }
      return 0;
    };

    U = new PriorityQueue<>(nodeComparator);
    gValues = new HashMap<>();
    rhsValues = new HashMap<>();
    nodeWrapperMap = new HashMap<>();
    for (int nodeId : graph.nodes.keySet()) {
      gValues.put(nodeId, Double.POSITIVE_INFINITY);
      rhsValues.put(nodeId, Double.POSITIVE_INFINITY);
    }

    rhsValues.put(goalId, 0.0);
    NodeWrapper goalWrapper = new NodeWrapper(graph.getNode(goalId), calculateKey(goalId));
    U.add(goalWrapper);
    nodeWrapperMap.put(goalId, goalWrapper);
  }


  private double[] calculateKey(int nodeId) {
    double min_g_rhs = Math.min(gValues.get(nodeId), rhsValues.get(nodeId));
    return new double[]{
        min_g_rhs + heuristic(graph.getNode(startId), graph.getNode(nodeId)),
        min_g_rhs};
  }

  private double heuristic(Node start, Node goal) {
//    return Math.sqrt(Math.pow(goal.x - start.x, 2) + Math.pow(goal.y - start.y, 2));
    return 0;
  }

  private void updateVertex(int nodeId) {
    if (nodeId != goalId && U.contains(nodeWrapperMap.get(nodeId))) {
      U.remove(nodeWrapperMap.get(nodeId));
      var newNodeWrapper = new NodeWrapper(graph.getNode(nodeId), calculateKey(nodeId));
      U.add(newNodeWrapper);
      nodeWrapperMap.put(nodeId, newNodeWrapper);
    } else if (gValues.get(nodeId) != rhsValues.get(nodeId) && !U.contains(
        nodeWrapperMap.get(nodeId))) {
      NodeWrapper newNodeWrapper = new NodeWrapper(graph.getNode(nodeId), calculateKey(nodeId));
      U.add(newNodeWrapper);
      nodeWrapperMap.put(nodeId, newNodeWrapper);
    } else if (gValues.get(nodeId) == rhsValues.get(nodeId) && U.contains(
        nodeWrapperMap.get(nodeId))) {
      U.remove(nodeWrapperMap.get(nodeId));
//      nodeWrapperMap.remove(nodeId);
    }
  }

  //    if (nodeId != goalId && U.contains(nodeWrapperMap.get(nodeId))) {
//      double minRhs = Double.POSITIVE_INFINITY;
//      for (Map.Entry<Integer, Double> entry : graph.getNeighbors(nodeId).entrySet()) {
//        int neighborId = entry.getKey();
//        double cost = entry.getValue();
//        minRhs = Math.min(minRhs, cost + gValues.get(neighborId));
//      }
//      rhsValues.put(nodeId, minRhs);
//    }
//
//    NodeWrapper nodeWrapper = nodeWrapperMap.get(nodeId);
//    if (nodeWrapper != null) {
//      U.remove(nodeWrapper);
//      nodeWrapperMap.remove(nodeId);
//    }
//
//    if (!gValues.get(nodeId).equals(rhsValues.get(nodeId))) {
//      NodeWrapper newNodeWrapper = new NodeWrapper(graph.getNode(nodeId), calculateKey(nodeId));
//      U.add(newNodeWrapper);
//      nodeWrapperMap.put(nodeId, newNodeWrapper);
//    }
  public void updateEdge(int node1, int node2, double newWeight) {
    //edge1

    int u = node1;
    int v = node2;
    if (!graph.edgeExists(u, v)) {
      graph.addEdge(u, v, Double.POSITIVE_INFINITY);
//      throw new RuntimeException("Edge " + u + "-" + v + " does not exist");
      // IDK if this is nessesary
      NodeWrapper newNodeWrapper = new NodeWrapper(graph.getNode(u), calculateKey(u));
      U.add(newNodeWrapper);
      nodeWrapperMap.put(u, newNodeWrapper);
      NodeWrapper newNodeWrapper2 = new NodeWrapper(graph.getNode(v), calculateKey(v));
      U.add(newNodeWrapper2);
      nodeWrapperMap.put(v, newNodeWrapper2);
      // IDK if this is nessesary
    }
    double oldWeight = graph.getEdgeWeight(u, v);
    graph.addEdge(u, v, newWeight);
    if (oldWeight > newWeight) {
      if (u != goalId) {
        rhsValues.put(u, Math.min(rhsValues.get(u), newWeight + gValues.get(v)));
      }

    } else if (rhsValues.get(u) == oldWeight + gValues.get(v)) {
      if (u != goalId) {
        var minRhs = Double.POSITIVE_INFINITY;
        for (Map.Entry<Integer, Double> entry : graph.getSuccedors(u).entrySet()) {
          int neighborId = entry.getKey();
          double cost = graph.getEdgeWeight(u, neighborId);
          minRhs = Math.min(minRhs, cost + gValues.get(neighborId));
        }
        rhsValues.put(u, minRhs);
      }
    }
    updateVertex(u);

//    computeShortestPath();

//    //edge2
//    {
//      int u = node2;
//      int v = node1;
//      double oldWeight = graph.getEdgeWeight(u, v);
//      graph.addEdge(u, v, newWeight);
//      if (oldWeight > newWeight) {
//        if (u != goalId) {
//          rhsValues.put(u, Math.min(rhsValues.get(u), newWeight + gValues.get(v)));
//        }
//
//      } else if (rhsValues.get(u) == oldWeight + gValues.get(v)) {
//        if (u != goalId) {
//          var minRhs = Double.POSITIVE_INFINITY;
//          for (Map.Entry<Integer, Double> entry : graph.getSuccedors(u).entrySet()) {
//            int neighborId = entry.getKey();
//            double cost = graph.getEdgeWeight(u, neighborId);
//            minRhs = Math.min(minRhs, cost + gValues.get(neighborId));
//          }
//          rhsValues.put(u, minRhs);
//        }
//        updateVertex(u);
//      }
//    }
  }


  public List<Node> computeShortestPath() {
//    while (!U.isEmpty() &&
//        (nodeComparator.compare(U.peek(), nodeWrapperMap.get(startId)) < 0 ||
//            !rhsValues.get(startId).equals(gValues.get(startId)))) {
    while (!U.isEmpty() &&
        (Arrays.compare(U.peek().key, calculateKey(startId)) < 0 ||
            rhsValues.get(startId) != gValues.get(startId))) {
      NodeWrapper u = U.peek();
      double k_old[] = u.key;
      double k_new[] = calculateKey(u.node.id);
      if (Arrays.compare(k_old, k_new) < 0) {
        u.key = k_new;
      } else if (gValues.get(u.node.id) > rhsValues.get(u.node.id)) {
        gValues.put(u.node.id, rhsValues.get(u.node.id));
        U.remove(u);
        for (var s : graph.getPredecessors(u.node.id)) {
          if (s != goalId) {
            rhsValues.put(s, Math.min(rhsValues.get(s),
                graph.getEdgeWeight(s, u.node.id) + gValues.get(u.node.id)));
          }
          updateVertex(s);
        }
      } else {
        double g_old = gValues.get(u.node.id);
        gValues.put(u.node.id, Double.POSITIVE_INFINITY);
        for (var s : graph.getPredecessors(u.node.id)) {
          if (rhsValues.get(s) == graph.getEdgeWeight(s, u.node.id) + g_old) {
            if (s != goalId) {
              var minRhs = Double.POSITIVE_INFINITY;
              for (Map.Entry<Integer, Double> sp : graph.getSuccedors(s).entrySet()) {
                int neighborId = sp.getKey();
                double cost = graph.getEdgeWeight(s, neighborId);
                minRhs = Math.min(minRhs, cost + gValues.get(neighborId));
              }
              rhsValues.put(s, minRhs);
            }
            updateVertex(s);
          }
        }

        if (u.node.id != goalId) {
          var minRhs = Double.POSITIVE_INFINITY;
          for (Map.Entry<Integer, Double> sp : graph.getSuccedors(u.node.id).entrySet()) {
            int neighborId = sp.getKey();
            double cost = graph.getEdgeWeight(u.node.id, neighborId);
            minRhs = Math.min(minRhs, cost + gValues.get(neighborId));
          }
        }
        updateVertex(u.node.id);
      }
    }

    // Reconstruct the shortest path
    List<Node> shortestPath = new ArrayList<>();
    int currentNodeId = startId;
    while (currentNodeId != goalId) {
      shortestPath.add(graph.getNode(currentNodeId));
      int nextNodeId = -1;
      double minCost = Double.POSITIVE_INFINITY;
      for (Map.Entry<Integer, Double> entry : graph.getSuccedors(currentNodeId).entrySet()) {
        int neighborId = entry.getKey();
        double cost = entry.getValue();
        double newPathCost = cost + gValues.get(neighborId);
        if (newPathCost < minCost) {
          minCost = newPathCost;
          nextNodeId = neighborId;
        }
      }
      if (nextNodeId == -1) {
        System.out.println("No valid path to the goal found.");
        return new ArrayList<>();
      }
      currentNodeId = nextNodeId;
    }
    shortestPath.add(graph.getNode(goalId));

    return shortestPath;
  }

//  { while (!U.isEmpty() &&
//      (nodeComparator.compare(U.peek().key, calculateKey(startId)) < 0 ||
//          !rhsValues.get(startId).equals(gValues.get(startId)))) {
//    NodeWrapper currentNodeWrapper = U.poll();
//    int currentNodeId = currentNodeWrapper.node.id;
//    System.out.println("Processing node: " + currentNodeId);
//    if (gValues.get(currentNodeId) > rhsValues.get(currentNodeId)) {
//      gValues.put(currentNodeId, rhsValues.get(currentNodeId));
//
//      for (int neighborId : graph.getNeighbors(currentNodeId).keySet()) {
//        updateVertex(neighborId);
//      }
//    } else {
//      gValues.put(currentNodeId, Double.POSITIVE_INFINITY);
//      for (int neighborId : graph.getNeighbors(currentNodeId).keySet()) {
//        updateVertex(neighborId);
//      }
//      updateVertex(currentNodeId);
//    }
//  }
//
//    // Reconstruct the shortest path
//    List<Node> shortestPath = new ArrayList<>();
//    int currentNodeId = startId;
//    while (currentNodeId != goalId) {
//      shortestPath.add(graph.getNode(currentNodeId));
//      int nextNodeId = -1;
//      double minCost = Double.POSITIVE_INFINITY;
//      for (Map.Entry<Integer, Double> entry : graph.getNeighbors(currentNodeId).entrySet()) {
//        int neighborId = entry.getKey();
//        double cost = entry.getValue();
//        double newPathCost = cost + gValues.get(neighborId);
//        if (newPathCost < minCost) {
//          minCost = newPathCost;
//          nextNodeId = neighborId;
//        }
//      }
//      if (nextNodeId == -1) {
//        System.out.println("No valid path to the goal found.");
//        return new ArrayList<>();
//      }
//      currentNodeId = nextNodeId;
//    }
//    shortestPath.add(graph.getNode(goalId));
//
//    return shortestPath;}

  public double getShortestPathCost(List<Node> shortestPath) {
    double cost = 0;
    for (int i = 0; i < shortestPath.size() - 1; i++) {
      Node currentNode = shortestPath.get(i);
      Node nextNode = shortestPath.get(i + 1);
      cost += graph.getEdgeWeight(currentNode.id, nextNode.id);
    }
    return cost;
  }

  private static class NodeWrapper implements Comparable<NodeWrapper> {

    Node node;
    double[] key;

    public NodeWrapper(Node node, double[] key) {
      this.node = node;
      this.key = key;
    }

    @Override
    public int compareTo(NodeWrapper o) {
      return Arrays.compare(this.key, o.key);
    }
  }
}

//import java.util.ArrayList;
//import java.util.Comparator;
//import java.util.HashMap;
//import java.util.List;
//import java.util.Map;
//import java.util.PriorityQueue;
//
//public class DStarLite {
//
//  Graph graph;
//  int startId;
//  int goalId;
//
//  // Algorithm data structures
//  PriorityQueue<Integer> U;
//  Map<Integer, Double> gValues;
//  Map<Integer, Double> rhsValues;
//  Comparator<Integer> nodeComparator;
//
//  public DStarLite(Graph graph, int startId, int goalId) {
//    this.graph = graph;
//    this.startId = startId;
//    this.goalId = goalId;
//
//    initialize();
//  }
//
//  private void initialize() {
////    Comparator<Integer> nodeComparator = (nodeId1, nodeId2) -> Arrays.compare(calculateKey(nodeId1),
////        calculateKey(nodeId2));
//    nodeComparator = (nodeId1, nodeId2) -> {
//      var key1 = calculateKey(nodeId1);
//      var key2 = calculateKey(nodeId2);
//      if (key1[0] < key2[0]) {
//        return -1;
//      } else if (key1[0] > key2[0]) {
//        return 1;
//      } else {
//        if (key1[1] < key2[1]) {
//          return -1;
//        } else if (key1[1] > key2[1]) {
//          return 1;
//        }
//      }
//      return 0;
//    };
//    U = new PriorityQueue<>(nodeComparator);
//    gValues = new HashMap<>();
//    rhsValues = new HashMap<>();
//
//    for (int nodeId : graph.nodes.keySet()) {
//      gValues.put(nodeId, Double.POSITIVE_INFINITY);
//      rhsValues.put(nodeId, Double.POSITIVE_INFINITY);
//    }
//
//    rhsValues.put(goalId, 0.0);
//    U.add(goalId);
//  }
//
//  private double[] calculateKey(int nodeId) {
//    double min_g_rhs = Math.min(gValues.get(nodeId), rhsValues.get(nodeId));
//    return new double[]{
//        min_g_rhs + heuristic(graph.getNode(startId), graph.getNode(nodeId)),
//        min_g_rhs};
//  }
//
//  private double heuristic(Node start, Node goal) {
//    return Math.sqrt(Math.pow(goal.x - start.x, 2) + Math.pow(goal.y - start.y, 2));
//  }
//
//  private void updateVertex(int nodeId) {
//    if (nodeId != goalId) {
//      double minRhs = Double.POSITIVE_INFINITY;
//      for (Map.Entry<Integer, Double> entry : graph.getNeighbors(nodeId).entrySet()) {
//        int neighborId = entry.getKey();
//        double cost = entry.getValue();
//        minRhs = Math.min(minRhs, cost + gValues.get(neighborId));
//      }
//      rhsValues.put(nodeId, minRhs);
//    }
//
//    U.remove(nodeId);
//
//    if (!gValues.get(nodeId).equals(rhsValues.get(nodeId))) {
//      U.add(nodeId);
//    }
//  }
//
//  public void updateEdge(int node1, int node2, double newWeight) {
//    graph.addEdge(node1, node2, newWeight);
////    graph.addEdge(node2, node1, newWeight);
//    updateVertex(node1);
////    updateVertex(node2);
//  }
//
//  public List<Node> computeShortestPath() {
////    while (!U.isEmpty() &&
////        (Arrays.compare(calculateKey(U.peek()), calculateKey(startId)) < 0 ||
////            !rhsValues.get(startId).equals(gValues.get(startId)))) {
////    while (!U.isEmpty() &&
////        (( nodeComparator.compare(U.peek(), startId) < 0) ||
////            !rhsValues.get(startId).equals(gValues.get(startId)))) {
//    while (true) {
//      var c1 = !U.isEmpty();
//      if(!c1) break;
//      var c2 = nodeComparator.compare(U.peek(), startId) < 0;
//      var c3 = !rhsValues.get(startId).equals(gValues.get(startId));
//      if ( !(c1 && (c2 || c3))) {
//        break;
//      }
//
//      int currentNodeId = U.poll();
//      System.out.println("Processing node: " + currentNodeId);
//      if (gValues.get(currentNodeId) > rhsValues.get(currentNodeId)) {
//        gValues.put(currentNodeId, rhsValues.get(currentNodeId));
//
//        for (int neighborId : graph.getPredecessors(currentNodeId)) {
//          if(currentNodeId!=goalId){
//            rhsValues.put(neighborId, Math.min(rhsValues.get(neighborId), graph.getEdgeWeight(neighborId, currentNodeId) + gValues.get(neighborId)));
//          }
//          updateVertex(neighborId);
//        }
//      } else {
//        gValues.put(currentNodeId, Double.POSITIVE_INFINITY);
//        for (int neighborId : graph.getNeighbors(currentNodeId).keySet()) {
//          updateVertex(neighborId);
//        }
//        updateVertex(currentNodeId);
//      }
//    }
//
//    // Reconstruct the shortest path
//    List<Node> shortestPath = new ArrayList<>();
//    int currentNodeId = startId;
//    while (currentNodeId != goalId) {
//      shortestPath.add(graph.getNode(currentNodeId));
//      int nextNodeId = -1;
//      double minCost = Double.POSITIVE_INFINITY;
//      for (Map.Entry<Integer, Double> entry : graph.getNeighbors(currentNodeId).entrySet()) {
//        int neighborId = entry.getKey();
//        double cost = entry.getValue();
//        double newPathCost = cost + gValues.get(neighborId);
//        if (newPathCost < minCost) {
//          minCost = newPathCost;
//          nextNodeId = neighborId;
//        }
//      }
//      if (nextNodeId == -1) {
//        System.out.println("No valid path to the goal found.");
//        return new ArrayList<>();
//      }
//      currentNodeId = nextNodeId;
//    }
//    shortestPath.add(graph.getNode(goalId));
//
//    return shortestPath;
//  }
//
//  public double getShortestPathCost(List<Node> shortestPath) {
//    double cost = 0;
//    for (int i = 0; i < shortestPath.size() - 1; i++) {
//      Node currentNode = shortestPath.get(i);
//      Node nextNode = shortestPath.get(i + 1);
//      cost += graph.getEdgeWeight(currentNode.id, nextNode.id);
//    }
//    return cost;
//  }
//    }