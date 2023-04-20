import java.util.Comparator;
import java.util.HashMap;
import java.util.HashSet;
import java.util.LinkedList;
import java.util.List;
import java.util.Map;
import java.util.PriorityQueue;
import java.util.Set;

public class Dijkstra {
  private Graph graph;

  public Dijkstra(Graph graph) {
    this.graph = graph;
  }

  public List<Node> computeShortestPath(int startId, int goalId) {
    Map<Integer, Double> distances = new HashMap<>();
    Map<Integer, Integer> previous = new HashMap<>();
    Set<Integer> visited = new HashSet<>();
    PriorityQueue<Node> queue = new PriorityQueue<>(Comparator.comparingDouble(node -> distances.get(node.id)));

    distances.put(startId, 0.0);
    queue.add(graph.getNode(startId));

    while (!queue.isEmpty()) {
      int currentId = queue.poll().id;
      visited.add(currentId);

      if (currentId == goalId) {
        break;
      }

      for (Map.Entry<Integer, Double> neighborEntry : graph.getSuccedors(currentId).entrySet()) {
        int neighborId = neighborEntry.getKey();
        if (visited.contains(neighborId)) {
          continue;
        }

        double newDistance = distances.get(currentId) + neighborEntry.getValue();
        if (!distances.containsKey(neighborId) || newDistance < distances.get(neighborId)) {
          distances.put(neighborId, newDistance);
          previous.put(neighborId, currentId);
          queue.remove(graph.getNode(neighborId));
          queue.add(graph.getNode(neighborId));
        }
      }
    }

    if (!previous.containsKey(goalId)) {
      return null;
    }

    LinkedList<Node> path = new LinkedList<>();
    int nodeId = goalId;
    while (nodeId != startId) {
      path.addFirst(graph.getNode(nodeId));
      nodeId = previous.get(nodeId);
    }
    path.addFirst(graph.getNode(startId));

    return path;
  }

  public double getShortestPathCost(List<Node> shortestPath) {
    double cost = 0;
    for (int i = 0; i < shortestPath.size() - 1; i++) {
      Node currentNode = shortestPath.get(i);
      Node nextNode = shortestPath.get(i + 1);
      cost += graph.getEdgeWeight(currentNode.id, nextNode.id);
    }
    return cost;
  }

}
