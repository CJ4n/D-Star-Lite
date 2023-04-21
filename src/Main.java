import java.util.List;

public class Main {

  public static void main(String[] args) {
    Graph graph = getGraph();

    int startId = 0;
    int goalId = 29;
    DStarLite dstarLite = new DStarLite(graph, startId, goalId);
    Dijkstra dijkstra = new Dijkstra(graph);
    calculateNewPathWithBothAlgorithms(startId, goalId, graph, dstarLite, dijkstra);

    dstarLite.updateEdge(10, 27, 1);
    dstarLite.updateEdge(20, 29, 20);
    dstarLite.updateEdge(0, 29, 0);
    calculateNewPathWithBothAlgorithms(startId, goalId, graph, dstarLite, dijkstra);
//
    dstarLite.updateEdge(1, 8, 0.5);
    dstarLite.updateEdge(27, 28, 10);
    dstarLite.updateEdge(9, 17, 0);
    dstarLite.updateEdge(1, 28, 10);
    dstarLite.updateEdge(2, 28, 10);
    dstarLite.updateEdge(3, 28, 10);
    dstarLite.updateEdge(3, 28, 2);
    dstarLite.updateEdge(28, 29, 2);
    dstarLite.updateEdge(4, 5, 20);
    calculateNewPathWithBothAlgorithms(startId, goalId, graph, dstarLite, dijkstra);

  }

  private static Graph getGraph() {
    Graph graph = new Graph();

    graph.addNode(0, 0, 0);
    graph.addNode(1, 1, 0);
    graph.addNode(2, 2, 0);
    graph.addNode(3, 3, 0);
    graph.addNode(4, 4, 0);
    graph.addNode(5, 0, 1);
    graph.addNode(6, 1, 1);
    graph.addNode(7, 2, 1);
    graph.addNode(8, 3, 1);
    graph.addNode(9, 4, 1);
    graph.addNode(10, 0, 2);
    graph.addNode(11, 1, 2);
    graph.addNode(12, 2, 2);
    graph.addNode(13, 3, 2);
    graph.addNode(14, 4, 2);
    graph.addNode(15, 0, 3);
    graph.addNode(16, 1, 3);
    graph.addNode(17, 2, 3);
    graph.addNode(18, 3, 3);
    graph.addNode(19, 4, 3);
    graph.addNode(20, 0, 4);
    graph.addNode(21, 1, 4);
    graph.addNode(22, 2, 4);
    graph.addNode(23, 3, 4);
    graph.addNode(24, 4, 4);
    graph.addNode(25, 5, 0);
    graph.addNode(26, 5, 1);
    graph.addNode(27, 5, 2);
    graph.addNode(28, 5, 3);
    graph.addNode(29, 5, 4);

    // Add edges
    graph.addEdge(0, 1, 1);
    graph.addEdge(0, 5, 1);
    graph.addEdge(1, 2, 1);
    graph.addEdge(1, 6, 1);
    graph.addEdge(2, 3, 1);
    graph.addEdge(2, 7, 1);
    graph.addEdge(3, 4, 1);
    graph.addEdge(3, 8, 1);
    graph.addEdge(4, 9, 1);
    graph.addEdge(5, 6, 1);
    graph.addEdge(5, 10, 1);
    graph.addEdge(6, 7, 1);
    graph.addEdge(6, 11, 1);
    graph.addEdge(7, 8, 1);
    graph.addEdge(7, 12, 1);
    graph.addEdge(8, 9, 1);
    graph.addEdge(8, 13, 1);
    graph.addEdge(9, 14, 1);
    graph.addEdge(10, 11, 1);
    graph.addEdge(10, 15, 1);
    graph.addEdge(11, 12, 1);
    graph.addEdge(11, 16, 1);
    graph.addEdge(12, 13, 1);
    graph.addEdge(12, 17, 1);
    graph.addEdge(13, 14, 1);
    graph.addEdge(13, 18, 1);
    graph.addEdge(14, 19, 1);
    graph.addEdge(15, 16, 1);
    graph.addEdge(15, 20, 1);
    graph.addEdge(16, 17, 1);
    graph.addEdge(16, 21, 1);
    graph.addEdge(17, 18, 1);
    graph.addEdge(17, 22, 1);
    graph.addEdge(18, 19, 1);
    graph.addEdge(18, 23, 1);
    graph.addEdge(19, 24, 1);
    graph.addEdge(20, 21, 1);
    graph.addEdge(21, 22, 1);
    graph.addEdge(22, 23, 1);
    graph.addEdge(23, 24, 1);
    graph.addEdge(25, 26, 1);
    graph.addEdge(26, 27, 1);
    graph.addEdge(27, 28, 1);
    graph.addEdge(28, 29, 1);
    graph.addEdge(5, 26, 5);
    graph.addEdge(10, 27, 5);
    graph.addEdge(15, 28, 5);
    graph.addEdge(20, 29, 5);
    graph.addEdge(3, 28, 2);
    graph.addEdge(1, 28, 22);
//    graph.addEdge(10, 27, 5123);
    return graph;
  }

  private static Graph getGraph2() {
    Graph graph = new Graph();

    graph.addNode(0, 0, 0);
    graph.addNode(1, 1, 0);
    graph.addNode(2, 2, 0);
    graph.addNode(3, 3, 0);
    graph.addNode(4, 4, 0);
    graph.addNode(5, 0, 1);

    graph.addEdge(0,1,1);
    graph.addEdge(1,2,1);
    graph.addEdge(2,3,1);
    graph.addEdge(3,4,1);
    graph.addEdge(4,5,1);
    return graph;
  }

  private static void calculateNewPathWithBothAlgorithms(int startId, int goalId, Graph graph,
      DStarLite dstarLite, Dijkstra dijkstra) {
    double dStarLitePathCost = findAndPrintShortestPathDStarLite(dstarLite, graph);
    double dijkstraPathCost = findAndPrintShortestPathDijkstra(dijkstra, graph, startId, goalId);
    if (dStarLitePathCost != dijkstraPathCost) {
      System.out.println("DStarLite and Dijkstra path cost are not equal");
      throw new RuntimeException("DStarLite and Dijkstra path cost are not equal");
    }
    System.out.println("---------------------------------------");

  }

  private static double findAndPrintShortestPathDStarLite(DStarLite dstarLite, Graph graph) {
    List<Node> shortestPath = dstarLite.computeShortestPath();
    double pathCost = dstarLite.getShortestPathCost(shortestPath);
    System.out.println("DStarLite Shortest path cost: " + pathCost);
    System.out.println("DStarLite Shortest path:");

    printPath(graph, shortestPath);
    return pathCost;
  }

  private static double findAndPrintShortestPathDijkstra(Dijkstra dijkstra, Graph graph, int start,
      int goal) {
    List<Node> shortestPath = dijkstra.computeShortestPath(start, goal);
    double pathCost = dijkstra.getShortestPathCost(shortestPath);
    System.out.println("Dijkstra Shortest path cost: " + pathCost);
    System.out.println("Dijkstra Shortest path: ");
    printPath(graph, shortestPath);
    return pathCost;
  }

  private static void printPath(Graph graph, List<Node> shortestPath) {
    for (int i = 0; i < shortestPath.size(); i++) {
      Node cur = shortestPath.get(i);
      System.out.println(cur);
      if (i + 1 < shortestPath.size()) {
        Node next = shortestPath.get(i + 1);
        System.out.println("Edge weight: " + graph.getEdgeWeight(cur.id, next.id));
      }
    }
  }
}
