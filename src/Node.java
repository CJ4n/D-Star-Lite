public class Node {
  int id;
  double x;
  double y;

  public Node(int id, double x, double y) {
    this.id = id;
    this.x = x;
    this.y = y;
  }

  @Override
  public String toString() {
//    return "Node{" + "id=" + id + ", x=" + x + ", y=" + y + '}';
    return "Node id = " + id;

  }
}