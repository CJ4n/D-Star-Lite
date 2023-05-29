import java.io.InputStream;
import java.util.HashMap;
import java.util.Map;
import java.util.Scanner;

public class BatchLoader {
    Graph graph = new Graph();
    DStarLite dStarLite;
    Dijkstra dijkstra;

    public BatchLoader(String[] args) {

    }

    public void loadFromStdio() {
        loadFromStream(System.in);
    }

    public void loadFromStream(InputStream stream) {
        Scanner scanner = new Scanner(stream);

        loadGraph(scanner);

        this.dStarLite = new DStarLite(graph, 0, graph.nodes.size() - 1);
        this.dijkstra = new Dijkstra(graph);

        loadAndPerformOperations(scanner);
    }

    public void loadGraph(Scanner scanner) {
        int numVertices = scanner.nextInt();
        int numEdges = scanner.nextInt();

        for (int i = 0; i < numVertices; i++) {
            int x = scanner.nextInt();
            int y = scanner.nextInt();
            graph.addNode(i, x, y);
        }

        for (int i = 0; i < numEdges; i++) {
            int u = scanner.nextInt();
            int v = scanner.nextInt();
            int weight = scanner.nextInt();
            graph.addEdge(u, v, weight);
        }
    }

    public void loadAndPerformOperations(Scanner scanner) {
        int numOperations = scanner.nextInt();

        for (int i = 0; i < numOperations; i++) {
            String operation = scanner.next();
            int u = scanner.nextInt();
            int v = scanner.nextInt();

            switch (operation) {
                case "insert" -> {

                    int weight = scanner.nextInt();
                    //dStarLite.updateEdge(u, v, weight);
                    dijkstra.computeShortestPath(0, graph.nodes.size() - 1);
                }
                case "delete" -> {
                    //dStarLite.updateEdge(u, v, Integer.MAX_VALUE);
                    graph.addEdge(u, v, Integer.MAX_VALUE);
                    graph.edges.get(u).remove(v);
                }
                default -> System.out.println("Invalid operation: " + operation);
            }
        }
    }
}
