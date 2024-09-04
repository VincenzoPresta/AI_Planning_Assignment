package fr.uga.pddl4j.examples.asp;
import fr.uga.pddl4j.examples.asp.CheckerBoxContent;
import fr.uga.pddl4j.problem.State;

/**
 * This class implements a node of the tree search.
 */
public class Node extends State {

    private Node parent;
    private int action;
    private double cost;
    private double heuristic;
    private int depth;

    public Node(State state) {
        super(state);
    }

    public Node(State state, Node parent, int action, double cost, double heuristic) {
        super(state);
        this.parent = parent;
        this.action = action;
        this.cost = cost;
        this.heuristic = heuristic;
        this.depth = -1;
    }

    public int getAction() {
        return this.action;
    }

    public void setAction(final int action) {
        this.action = action;
    }

    public final Node getParent() {
        return parent;
    }

    public final void setParent(Node parent) {
        this.parent = parent;
    }

    public final double getCost() {
        return cost;
    }

    public final void setCost(double cost) {
        this.cost = cost;
    }

    public final double getHeuristic() {
        return heuristic;
    }

    public final void setHeuristic(double heuristic) {
        this.heuristic = heuristic;
    }

    public int getDepth() {
        return this.depth;
    }

    public void setDepth(final int depth) {
        this.depth = depth;
    }

    public final double getValueF(double weight) {
        return weight * this.heuristic + this.cost;
    }

    @Override
    public String toString() {
        return "Node{" +
                "parent=" + (parent != null ? parent.hashCode() : "null") +
                ", action=" + action +
                ", cost=" + cost +
                ", heuristic=" + heuristic +
                ", depth=" + depth +    
                ", state=" + super.toString() +
                '}';
    }
}
