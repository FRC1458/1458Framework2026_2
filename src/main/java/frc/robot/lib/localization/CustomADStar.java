package frc.robot.lib.localization;

import java.util.*;
import java.util.stream.Stream;
/** DO NOT USE */
public class CustomADStar {
    private static final double EPSILON0 = 2.5;
    private static final double EPSILON_MIN = 1.0;
    private static final double RAMP_RATE = 0.5;

    private static class Node implements Comparable<Node> {
        final int x, y;
        double g = Double.POSITIVE_INFINITY;
        double rhs = Double.POSITIVE_INFINITY;
        double h = 0.0;
        Node parent = null;
        boolean inClosed = false;
        boolean inOpen = false;
        boolean incons = false;
        boolean isObstacle = false;  

        Key key;

        public static class Key implements Comparable<Key> {
            final double first, second;
            public Key(double f, double s) {
                first = f; second = s;
            }
            @Override
            public int compareTo(Key o) {
                if (first > o.first) return +1;
                if (first < o.first) return -1;
                return Double.compare(second, o.second);
            }
        }

        public Node(int x, int y) {
            this.x = x;
            this.y = y;
        }

        public Key calculateKey(double eps, Node goal) {
            this.h = Math.hypot(this.x - goal.x, this.y - goal.y);

            double k2 = Math.min(this.g, this.rhs);
            double k1 = k2 + eps * this.h;
            this.key = new Key(k1, k2);
            return this.key;
        }

        @Override
        public int compareTo(Node o) {
            return this.key.compareTo(o.key);
        }

        @Override
        public boolean equals(Object o) {
            if (this == o) return true;
            if (!(o instanceof Node)) return false;
            Node other = (Node) o;
            return this.x == other.x && this.y == other.y;
        }

        @Override
        public int hashCode() {
            return Objects.hash(x, y);
        }
    }

    private static class Graph {
        final int width, height;
        final Node[][] matrix;
        final Node[] allNodes;

        public Graph(int width, int height) {
            this.width = width;
            this.height = height;
            matrix = new Node[width][height];

            List<Node> tempList = new ArrayList<>(width * height);
            for (int ix = 0; ix < width; ix++) {
                for (int iy = 0; iy < height; iy++) {
                    matrix[ix][iy] = new Node(ix, iy);
                    tempList.add(matrix[ix][iy]);
                }
            }
            allNodes = tempList.toArray(new Node[0]);
        }

        public void setObstacle(Node u) {
            u.isObstacle = true;
        }

        public void clearObstacle(Node u) {
            u.isObstacle = false;
        }

        public List<Node> getSuccessors(Node u) {
            List<Node> succs = new ArrayList<>(8);

            for (int dx = -1; dx <= 1; dx++) {
                for (int dy = -1; dy <= 1; dy++) {
                    if (dx == 0 && dy == 0) continue;
                    int nx = u.x + dx;
                    int ny = u.y + dy;
                    // Bounds check
                    if (nx < 0 || nx >= width || ny < 0 || ny >= height) {
                        continue;
                    }
                    Node v = matrix[nx][ny];
                    if (v.isObstacle) {
                        continue;
                    }
                    if (Math.abs(dx) == 1 && Math.abs(dy) == 1) {
                        Node corner1 = matrix[u.x + dx][u.y];
                        Node corner2 = matrix[u.x][u.y + dy];
                        if (corner1.isObstacle || corner2.isObstacle) {
                            continue;
                        }
                    }
                    succs.add(v);
                }
            }
            return succs;
        }

        public List<Node> getPredecessors(Node u) {
            return getSuccessors(u);
        }

        public double cost(Node u, Node v) {
            if (u.isObstacle || v.isObstacle) {
                return Double.POSITIVE_INFINITY;
            }
            return Math.hypot(u.x - v.x, u.y - v.y);
        }

        public double heuristic(Node u, Node v) {
            return Math.hypot(u.x - v.x, u.y - v.y);
        }
    }

    private final Graph graph;
    private final Node start, goal;
    private final PriorityQueue<Node> OPEN = new PriorityQueue<>();
    private final LinkedList<Node> INCONS = new LinkedList<>();
    private double epsilon = EPSILON0;

    public CustomADStar(Graph g, Node s, Node t) {
        this.graph = g;
        this.start = s;
        this.goal = t;

        for (Node u : g.allNodes) {
            u.g = Double.POSITIVE_INFINITY;
            u.rhs = Double.POSITIVE_INFINITY;
            u.parent = null;
            u.inOpen = false;
            u.inClosed = false;
            u.incons = false;
            u.h = g.heuristic(u, goal);
        }

        goal.rhs = 0.0;
        goal.calculateKey(epsilon, goal);
        OPEN.add(goal);
        goal.inOpen = true;

        computePath();

        improvePath();
    }


    private void updateNode(Node u) {
        if (u != goal) {
            double best = Double.POSITIVE_INFINITY;
            Node bestSucc = null;
            for (Node succ : graph.getSuccessors(u)) {
                double tentative = graph.cost(u, succ) + succ.g;
                if (tentative < best) {
                    best = tentative;
                    bestSucc = succ;
                }
            }
            u.rhs = best;
            u.parent = bestSucc;
        }

        if (u.inOpen) {
            OPEN.remove(u);
            u.inOpen = false;
        }

        if (u.g != u.rhs) {
            if (!u.inClosed) {
                u.calculateKey(epsilon, goal);
                OPEN.add(u);
                u.inOpen = true;
            } else {
                u.incons = true;
                INCONS.add(u);
            }
        }
    }

    private void computePath() {
        start.calculateKey(epsilon, goal);

        while (!OPEN.isEmpty()) {
            Node top = OPEN.peek();
            if (top.key.compareTo(start.key) >= 0 && start.rhs == start.g) {
                return;
            }

            top = OPEN.poll();
            top.inOpen = false;
            top.inClosed = true;

            Node.Key oldKey = top.key;
            Node.Key newKey = top.calculateKey(epsilon, goal);

            if (oldKey.compareTo(newKey) < 0) {
                OPEN.add(top);
                top.inOpen = true;
            }
            else if (top.g > top.rhs) {
                top.g = top.rhs;
                for (Node pred : graph.getPredecessors(top)) {
                    updateNode(pred);
                }
            }
            else {
                top.g = Double.POSITIVE_INFINITY;
                updateNode(top);
                for (Node pred : graph.getPredecessors(top)) {
                    updateNode(pred);
                }
            }
        }
    }

    private void improvePath() {
        while (epsilon > EPSILON_MIN) {
            epsilon = Math.max(EPSILON_MIN, epsilon - RAMP_RATE);

            List<Node> temp = new ArrayList<>(INCONS);
            INCONS.clear();
            for (Node u : temp) {
                u.calculateKey(epsilon, goal);
                u.inOpen = true;
                u.inClosed = false;
                OPEN.add(u);
                u.incons = false;
            }

            List<Node> allOpenNow = new ArrayList<>(OPEN);
            OPEN.clear();
            for (Node u : allOpenNow) {
                u.calculateKey(epsilon, goal);
                OPEN.add(u);
                u.inOpen = true;
            }

            for (Node u : graph.allNodes) {
                u.inClosed = false;
            }

            computePath();
        }
    }

    public void addObstacle(Node u) {
        if (!u.isObstacle) {
            u.isObstacle = true;
            resetAll();
            computePath();
            improvePath();
        }
    }

    public void removeObstacle(Node u) {
        if (u.isObstacle) {
            u.isObstacle = false;
            resetAll();
            computePath();
            improvePath();
        }
    }

    private void resetAll() {
        epsilon = EPSILON0;
        OPEN.clear();
        INCONS.clear();

        for (Node u : graph.allNodes) {
            u.g = Double.POSITIVE_INFINITY;
            u.rhs = Double.POSITIVE_INFINITY;
            u.parent = null;
            u.inOpen = false;
            u.inClosed = false;
            u.incons = false;
            u.h = graph.heuristic(u, goal);
        }

        goal.rhs = 0.0;
        goal.calculateKey(epsilon, goal);
        OPEN.add(goal);
        goal.inOpen = true;
    }

    public List<Node> extractPath() {
        List<Node> path = new ArrayList<>();
        Node u = start;
        if (start.parent == null && start != goal) {
            return path;
        }
        path.add(u);
        while (u != goal) {
            u = u.parent;
            if (u == null) {
                return new ArrayList<>();
            }
            path.add(u);
            if (path.size() > graph.width * graph.height) {
                break;
            }
        }
        return path;
    }

    // ────────────────────────────────────────────────────────────────────────────
    // For convenience, expose a method to find the “closest non‐obstacle” node
    // (mimicking LocalADStar’s findClosestNonObstacle).  This does a simple BFS.
    // ────────────────────────────────────────────────────────────────────────────
    public Node findClosestNonObstacle(int startX, int startY) {
        if (startX < 0 || startX >= graph.width || startY < 0 || startY >= graph.height) {
            return null;
        }
        Node root = graph.matrix[startX][startY];
        if (!root.isObstacle) {
            return root;
        }

        boolean[][] visited = new boolean[graph.width][graph.height];
        Queue<Node> queue = new LinkedList<>();
        queue.add(root);
        visited[root.x][root.y] = true;

        while (!queue.isEmpty()) {
            Node n = queue.poll();
            for (Node nbr : graph.getSuccessors(n)) {
                if (!visited[nbr.x][nbr.y]) {
                    if (!nbr.isObstacle) {
                        return nbr;
                    }
                    visited[nbr.x][nbr.y] = true;
                    queue.add(nbr);
                }
            }
        }
        return null;
    }


    // ────────────────────────────────────────────────────────────────────────────
    // (Optional) If you want to call updateEdge(u,v,newCost) like before, you still can;
    // but any time newCost == ∞, you might simply do addObstacle(u) or addObstacle(v).
    // We'll leave updateEdge intact if you want fine‐grained control:
    // ────────────────────────────────────────────────────────────────────────────
    public void updateEdge(Node u, Node v, double newCost) {
        // If newCost is ∞, block both endpoints:
        if (newCost == Double.POSITIVE_INFINITY) {
            addObstacle(u);
            addObstacle(v);
            return;
        }
        // Otherwise, we would store it in an edgeCostMap (not shown), or just ignore
        // since we’re using node‐based obstacles now. For simplicity, we ignore non‐∞ updates.
        // If you do want custom edge costs, re‐introduce an edgeCostMap here.

        // After changing obstacle status, replan from scratch:
        resetAll();
        computePath();
        improvePath();
    }
}
