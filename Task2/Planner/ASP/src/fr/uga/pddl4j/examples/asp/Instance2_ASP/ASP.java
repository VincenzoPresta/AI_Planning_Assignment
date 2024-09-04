package fr.uga.pddl4j.examples.asp.Instance2_ASP;

import fr.uga.pddl4j.heuristics.state.StateHeuristic;
import fr.uga.pddl4j.parser.DefaultParsedProblem;
import fr.uga.pddl4j.parser.RequireKey;
import fr.uga.pddl4j.plan.Plan;
import fr.uga.pddl4j.plan.SequentialPlan;
import fr.uga.pddl4j.planners.AbstractPlanner;
import fr.uga.pddl4j.planners.Planner;
import fr.uga.pddl4j.planners.PlannerConfiguration;
import fr.uga.pddl4j.planners.ProblemNotSupportedException;
import fr.uga.pddl4j.planners.SearchStrategy;
import fr.uga.pddl4j.planners.statespace.search.StateSpaceSearch;
import fr.uga.pddl4j.problem.DefaultProblem;
import fr.uga.pddl4j.problem.Problem;
import fr.uga.pddl4j.problem.State;
import fr.uga.pddl4j.problem.Fluent;
import fr.uga.pddl4j.problem.operator.Action;
import fr.uga.pddl4j.problem.operator.ConditionalEffect;
import org.apache.logging.log4j.LogManager;
import org.apache.logging.log4j.Logger;
import picocli.CommandLine;
import fr.uga.pddl4j.examples.asp.Node;
import fr.uga.pddl4j.examples.asp.CheckerBoxContent;
import fr.uga.pddl4j.examples.asp.FortugnoGrecoPrestaHeuristic;
import fr.uga.pddl4j.parser.TypedSymbol;
import fr.uga.pddl4j.problem.Fluent;
import fr.uga.pddl4j.util.BitVector;
import java.lang.management.ManagementFactory;
import java.lang.management.MemoryMXBean;
import java.lang.management.MemoryUsage;



import java.io.*;
import java.util.*;
import java.util.Comparator;
import java.util.HashSet;
import java.util.List;
import java.util.PriorityQueue;
import java.util.Set;
import java.util.HashMap;
import java.util.LinkedList;


/**
 * The class is an example. It shows how to create a simple A* search planner able to
 * solve an ADL problem by choosing the heuristic to used and its weight.
 *
 * @author D. Pellier
 * @version 4.0 - 30.11.2021
 */
@CommandLine.Command(name = "ASP",
    version = "ASP 1.0",
    description = "Solves a specified planning problem using A* search strategy.",
    sortOptions = false,
    mixinStandardHelpOptions = true,
    headerHeading = "Usage:%n",
    synopsisHeading = "%n",
    descriptionHeading = "%nDescription:%n%n",
    parameterListHeading = "%nParameters:%n",
    optionListHeading = "%nOptions:%n")
public class ASP extends AbstractPlanner {

    HashMap<String,String> object_type = new HashMap<>(); //<Object, Type>
    HashMap<String, String> Types = new HashMap<>(); //<Content, ContentType>
    HashMap<String, Boolean> goalType = new HashMap<>(); //<Type, IsNeededForGoal>
    Set<String> goalWS = new HashSet<>(); //<Workstations needed for the goal>
    Set<Integer> rendundantContentState = new HashSet<>(); //<States with redundant content>
    Set<Integer> rendundantBoxState = new HashSet<>(); //<States with redundant boxes>
    HashMap<String, String> locationWS = new HashMap<>(); //<Location, Workstation>
    HashMap<Integer, String> Predicates = new HashMap<>(); //<PredicateValue, Predicate>
    HashMap<Integer, String> Constants = new HashMap<>();  //<ConstantValue, Constant>
    HashMap<String,LinkedList<String>> goal = new HashMap<>(); //<Workstation, ContentsNeededForTheGoal>
    Map<Node, CheckerBoxContent> nodeCheckerBoxMap = new HashMap<>(); //Saving CheckerBox content


    /**
     * The weight of the heuristic.
     */
    private double heuristicWeight;

    /**
     * The name of the heuristic used by the planner.
     */
    private StateHeuristic.Name heuristic;

    /**
     * The class logger.
     */
    private static final Logger LOGGER = LogManager.getLogger(ASP.class.getName());

     /**
     * Sets the weight of the heuristic.
     *
     * @param weight the weight of the heuristic. The weight must be greater than 0.
     * @throws IllegalArgumentException if the weight is strictly less than 0.
     */
    @CommandLine.Option(names = {"-w", "--weight"}, defaultValue = "1.0",
        paramLabel = "<weight>", description = "Set the weight of the heuristic (preset 1.0).")
    public void setHeuristicWeight(final double weight) {
        if (weight <= 0) {
            throw new IllegalArgumentException("Weight <= 0");
        }
        this.heuristicWeight = weight;
    }

    /**
     * Set the name of heuristic used by the planner to the solve a planning problem.
     *
     * @param heuristic the name of the heuristic.
     */
    @CommandLine.Option(names = {"-e", "--heuristic"}, defaultValue = "FAST_FORWARD",
        description = "Set the heuristic : AJUSTED_SUM, AJUSTED_SUM2, AJUSTED_SUM2M, COMBO, "
            + "MAX, FAST_FORWARD SET_LEVEL, SUM, SUM_MUTEX (preset: FAST_FORWARD)")
    public void setHeuristic(StateHeuristic.Name heuristic) {
        this.heuristic = heuristic;
    }

    /**
     * Returns the name of the heuristic used by the planner to solve a planning problem.
     *
     * @return the name of the heuristic used by the planner to solve a planning problem.
     */
    public final StateHeuristic.Name getHeuristic() {
        return this.heuristic;
    }

    /**
     * Returns the weight of the heuristic.
     *
     * @return the weight of the heuristic.
     */
    public final double getHeuristicWeight() {
        return this.heuristicWeight;
    }
    
    /**
     * Instantiates the planning problem from a parsed problem.
     *
     * @param problem the problem to instantiate.
     * @return the instantiated planning problem or null if the problem cannot be instantiated.
     */
    @Override
    public Problem instantiate(DefaultParsedProblem problem) {
        final Problem pb = new DefaultProblem(problem);
        pb.instantiate();
        return pb;
    }

    /**
     * Search a solution plan to a specified domain and problem using A*.
     *
     * @param problem the problem to solve.
     * @return the plan found or null if no plan was found.
     */
    @Override
    public Plan solve(final Problem problem) {
  
        LOGGER.info("* Starting A* search \n");
        // Search a solution
        Plan plan = null;
        //Per poter tenere traccia della memoria usata dall'algoritmo
        MemoryMXBean memoryBean = ManagementFactory.getMemoryMXBean();
        MemoryUsage beforeHeapMemoryUsage = memoryBean.getHeapMemoryUsage();
        Long beforeUsedMemory = beforeHeapMemoryUsage.getUsed();

        //Per tenere traccia del tempo usato dall'algoritmo
        final long begin = System.currentTimeMillis();
        
        try {
            plan = this.astar(problem);
        } catch (ProblemNotSupportedException e) {
            LOGGER.error("Problem not supported: " + e.getMessage());
        }

        final long end = System.currentTimeMillis();
        MemoryUsage afterHeapMemoryUsage = memoryBean.getHeapMemoryUsage();
        long afterUsedMemory = afterHeapMemoryUsage.getUsed();

        long memoryUsedByCustomAstar = afterUsedMemory - beforeUsedMemory;
        // If a plan is found update the statistics of the planner and log search information
        if (plan != null) {
            LOGGER.info("* A* search succeeded\n");
            this.getStatistics().setTimeToSearch(end - begin);
            this.getStatistics().setMemoryUsedToSearch(memoryUsedByCustomAstar);
        } else {
            LOGGER.info("* A* search failed\n");
        }
        // Return the plan found or null if the search fails.
        return plan;
    }



    /**
     * Search a solution plan for a planning problem using an A* search strategy.
     *
     * @param problem the problem to solve.
     * @return a plan solution for the problem or null if there is no solution
     * @throws ProblemNotSupportedException if the problem to solve is not supported by the planner.
     */
    public Plan astar(Problem problem) throws ProblemNotSupportedException {

    // Creazione delle mappe per i simboli di predicati e costanti

    //Constants Mapping
    int index = 0;
    for (String s : problem.getConstantSymbols()) {
        Constants.put(index, s);
        index++;
    }

    //Predicates Mapping
    index = 0;
    for (String s : problem.getPredicateSymbols()) {
        Predicates.put(index, s);
        index++;
    }

    HashMap<String,Integer> Locations = new HashMap<>();
    int locationIndex=0;
    // Creation of object_type map
    for (TypedSymbol<String> o : problem.getParsedProblem().getConstants()) {
        String object = o.toString().split(" - ")[0];
        String type = o.toString().split(" - ")[1];
        object_type.put(object, type);
        if(type.equals("LOCATION")){
            Locations.put(object,locationIndex);
            locationIndex++;
        } 
    }


    // Problem Fluents full List
    List<Fluent> fluents = problem.getFluents();

    //Bit Vector of positive Fluents
    BitVector initialStateFluents = problem.getInitialState().getPositiveFluents();

    //Bit Vector of Goal Fluents
    BitVector goalFluents = problem.getGoal().getPositiveFluents();

    int initialStateBitVectorIndex = initialStateFluents.nextSetBit(0);
    int goalBitVectorIndex = goalFluents.nextSetBit(0);
    int indexFluent = 0; 

    //Creation of checkerBox, ready to fill up
    CheckerBoxContent checkerBoxContent = new CheckerBoxContent();

    //Temporary Map to check if the problem is solvable
    Map<String,Integer> numberOfType = new HashMap<>();

    //Iteration for all the Fluents of the problem
    for (Fluent f : fluents) {
        boolean isInInitialState = (initialStateBitVectorIndex == indexFluent);
        boolean isInGoalState = (goalBitVectorIndex == indexFluent);
        int simboloAzione = f.getSymbol();

        // Gestisci il fluente se è presente nello stato iniziale
        if (isInInitialState) {
            if (Predicates.get(simboloAzione).equals("at")) {
                int objectValue = f.getArguments()[0];
                int locationValue = f.getArguments()[1];

                String object = Constants.get(objectValue);
                String location = Constants.get(locationValue);

                if (object_type.get(object).equals("BOX")) {
                    checkerBoxContent.insert(object, location, "BOX");
                } else if (object_type.get(object).equals("CONTENT")) {
                    checkerBoxContent.insert(object, location, "CONTENT");
                }
                else if (object_type.get(object).equals("WORKSTATION"))
                    locationWS.put(object,location);   
            }
            if (Predicates.get(simboloAzione).equals("is-type")) {
                int objectValue = f.getArguments()[0];
                int typeValue = f.getArguments()[1];

                String object = Constants.get(objectValue);
                String type = Constants.get(typeValue);

                if(numberOfType.containsKey(type)){
                    int pre = numberOfType.get(type);
                    pre++;
                    numberOfType.put(type,pre);
                }
                else numberOfType.put(type,1);

                Types.put(object, type);
            }
            initialStateBitVectorIndex = initialStateFluents.nextSetBit(initialStateBitVectorIndex + 1);
        }

        // Managament of the fluents if its in the goal
        if (isInGoalState) {
            if (Predicates.get(simboloAzione).equals("workstation-has-type")) {
                String contentType = Constants.get(f.getArguments()[1]);
                String ws = Constants.get(f.getArguments()[0]);
                if(!goal.containsKey(ws)){
                    goal.put(ws,new LinkedList<>());
                }

                if(numberOfType.containsKey(contentType)) {
                    int pre = numberOfType.get(contentType);
                    pre--;
                    numberOfType.put(contentType,pre);
                }
                else   
                    numberOfType.put(contentType,-1);
                goal.get(ws).add(contentType);
                goalType.put(contentType, true);
                goalWS.add(ws);
            }
            goalBitVectorIndex = goalFluents.nextSetBit(goalBitVectorIndex + 1);
        }

        indexFluent++;
    }
    System.out.println(numberOfType);
    for(String s:numberOfType.keySet())
        if(numberOfType.get(s)<0){
            LOGGER.info("No Plan Found, because the are not sufficient resources \n");
            return null;
        }
    
    checkerBoxContent.setContentType(Types);

    // Check if the problem is supported by the planner
    if (!this.isSupported(problem)) {
        throw new ProblemNotSupportedException("Problem not supported");
    }

    final FortugnoGrecoPrestaHeuristic heuristic = new FortugnoGrecoPrestaHeuristic(problem);
    final State init = new State(problem.getInitialState());
    final Set<Node> close = new HashSet<>();
    final double weight = this.getHeuristicWeight();

    final PriorityQueue<Node> open = new PriorityQueue<>(100, new Comparator<Node>() {
            public int compare(Node n1, Node n2) {
                double f1 = weight * n1.getHeuristic() + n1.getCost();
                double f2 = weight * n2.getHeuristic() + n2.getCost();
                return Double.compare(f1, f2);
            }
    });

    //Creation of the first node (root) and its CheckeBoxMap which represent the initial state of box and content;
    final Node root = new Node(init, null, -1, 0, heuristic.estimate(init, problem.getGoal()));
    nodeCheckerBoxMap.put(root, checkerBoxContent);
    open.add(root);

    Plan plan = null;
    final int timeout = this.getTimeout() * 1000;
    long time = 0;

    // Start the search
    while (!open.isEmpty() && plan == null && time < timeout) {
        final Node current = open.poll();
        close.add(current);

        //Get the state of Content and Box
        CheckerBoxContent currentCheckerBox = nodeCheckerBoxMap.get(current);
    
        if (current.satisfy(problem.getGoal())) {
            return this.extractPlan(current, problem);
        } else {
            for (int i = 0; i < problem.getActions().size(); i++) {
                Action a = problem.getActions().get(i);
                
                if (a.isApplicable(current)) {

                    //Generate a new content only when the state changes
                    boolean modifiedState = false;
                    CheckerBoxContent newCheckerBoxContent;
                    if(a.getName().contains("pick-up") ||  a.getName().contains("deliver") || a.getName().contains("fill") || a.getName().contains("empty")){
                        newCheckerBoxContent = new CheckerBoxContent(currentCheckerBox);
                        modifiedState = true;
                    }
                    else { 
                        newCheckerBoxContent = currentCheckerBox;
                    }
                    
                    //Check if the action selected is promising or if it will generate a sub-optimal state
                    if(PromisingMove(a,newCheckerBoxContent)){
                        Node next = new Node(current);
                    
                        final List<ConditionalEffect> effects = a.getConditionalEffects();
                        for (ConditionalEffect ce : effects) {
                            if (current.satisfy(ce.getCondition())) {
                                next.apply(ce.getEffect());
                            }
                        }
                            
                            if(modifiedState){
                                if (a.getName().equals("pick-up-from-workstation")) {
                                    String obj = Constants.get(a.getValueOfParameter(3));
                                    String loc = Constants.get(a.getValueOfParameter(2));
                                    
                                    //Update the content and box state
                                    newCheckerBoxContent.updateBoxLocation(obj, loc, false);
                                    
                                    int hash = newCheckerBoxContent.getProcessedBoxHash();
                                    if(rendundantBoxState.contains(hash)) continue;
                                    else rendundantBoxState.add(hash);
                                } else if (a.getName().equals("pick-up-from-location")) {
                                    String obj = Constants.get(a.getValueOfParameter(1));
                                    String loc = Constants.get(a.getValueOfParameter(2));
                                    
                                    //Update the content and box state
                                    newCheckerBoxContent.updateBoxLocation(obj, loc, false);

                                    //Check if the state is rendundant
                                    int hash = newCheckerBoxContent.getProcessedBoxHash();
                                    if(rendundantBoxState.contains(hash)) continue;
                                    else rendundantBoxState.add(hash);
                                } 
                                
                                
                                
                                else if (a.getName().equals("deliver-to-workstation")) {
                                    String obj = Constants.get(a.getValueOfParameter(3));
                                    String loc = Constants.get(a.getValueOfParameter(2));

                                    //Check if the state is rendundant
                                    int hash = newCheckerBoxContent.getProcessedBoxHash();
                                    if(rendundantBoxState.contains(hash)) rendundantBoxState.remove(hash);
                                    
                                    //Update the content and box state
                                    newCheckerBoxContent.updateBoxLocation(obj, loc, true);
                                } 
                                else if (a.getName().equals("deliver-to-location")) {
                                    String obj = Constants.get(a.getValueOfParameter(2));
                                    String loc = Constants.get(a.getValueOfParameter(1));

                                    //Check if the state is rendundant
                                    int hash = newCheckerBoxContent.getProcessedBoxHash();
                                    if(rendundantBoxState.contains(hash)) rendundantBoxState.remove(hash);

                                    //Update the content and box state
                                    newCheckerBoxContent.updateBoxLocation(obj, loc, true);
                                }
                                
                                else if (a.getName().equals("fill-box-from-location")) {
                                    String box = Constants.get(a.getValueOfParameter(1));
                                    String obj = Constants.get(a.getValueOfParameter(2));
                                    String loc = Constants.get(a.getValueOfParameter(3));

                                    //Update the content and box state
                                    newCheckerBoxContent.updateContentLocation(box, loc,obj, false, false);
                                    
                                    //Check if the state is rendundant
                                    int hash = newCheckerBoxContent.getProcessedContentHash();
                                    if(rendundantContentState.contains(hash)) continue;
                                    else rendundantContentState.add(hash);
                                } 
                                else if (a.getName().equals("fill-box-from-workstation")) {
                                    String box = Constants.get(a.getValueOfParameter(1));
                                    String obj = Constants.get(a.getValueOfParameter(2));
                                    String loc = Constants.get(a.getValueOfParameter(5));
                                    
                                    //Update the content and box state
                                    newCheckerBoxContent.updateContentLocation(box, loc, obj, false, true);
                                    
                                    //Check if the state is rendundant
                                    int hash = newCheckerBoxContent.getProcessedContentHash();
                                    if(rendundantContentState.contains(hash)) continue;
                                    else rendundantContentState.add(hash);
                                }  
                                
                                else if (a.getName().equals("empty-box-workstation")) {  
                                    String box = Constants.get(a.getValueOfParameter(1));
                                    String obj = Constants.get(a.getValueOfParameter(2));
                                    String loc = Constants.get(a.getValueOfParameter(5));
                                    
                                    //Update the content and box state
                                    newCheckerBoxContent.updateContentLocation(box, loc, obj, true, true);
                                }
                                else if (a.getName().equals("empty-box-location")) {
                                    String box = Constants.get(a.getValueOfParameter(1));
                                    String obj = Constants.get(a.getValueOfParameter(2));
                                    String loc = Constants.get(a.getValueOfParameter(3));
                                    
                                    //Update the content and box state
                                    newCheckerBoxContent.updateContentLocation(box, loc, obj, true, false);
                                }
                            }

                            final double g = current.getCost() + 1;
                            if (!close.contains(next)) {
                                next.setCost(g);
                                next.setParent(current);
                                next.setAction(i);
                                next.setHeuristic(heuristic.customEstimate(next, problem.getGoal(), a, current.getHeuristic()));
                                open.add(next);
                                nodeCheckerBoxMap.put(next, newCheckerBoxContent);
                            }
                    }
                }
            }
        }
    }

    return plan;
}

    public boolean PromisingMove(Action a, CheckerBoxContent checkerBoxContent){
        
        //If a box is filled with a content which isn't requested in the goal -> remove the move
        if(a.getName().contains("fill-box-from-location")){
            String object = Constants.get(a.getValueOfParameter(2));
            String type = Types.get(object);
            return goalType.containsKey(type);
        }

         else if(a.getName().contains("fill-box-from-workstation")){
            String type = Constants.get(a.getValueOfParameter(4));
            return goalType.containsKey(type);
        }

        //emptying a box in a workstation that DOESN'T need that content type is a wrong move
        else if(a.getName().contains("empty-box-workstation")){
            String workstation = Constants.get(a.getValueOfParameter(4));
            String leavingContent = Constants.get(a.getValueOfParameter(2));
            //If its leaving a goal content in the wrong workstation the move is not logical
            boolean isAGoalContent = goalType.get(Types.get(leavingContent)); 
            if(goal.get(workstation).contains(Types.get(leavingContent))) return true;
            return false;
        }

        //emptying a box which has a contentype needed in goal is a wrong move move
        //emptying a box in a location that doesn't have a goal content is a wrong move
        else if(a.getName().contains("empty-box-location") ){
            String location = Constants.get(a.getValueOfParameter(3));
            String leavingContent = Constants.get(a.getValueOfParameter(2));
            boolean isAGoalContent = goalType.containsKey(Types.get(leavingContent)); //If its removing a goal content the move is not logical
            if(!isAGoalContent){
                //If the content removed from the box is not a goal content, 
                //there must be at least one other content at the location that has a goal content type.
                if(checkerBoxContent.contentLocation.containsKey(location)){ 
                    List<String> contentInLocation = checkerBoxContent.contentLocation.get(location);
                    for(String content : contentInLocation){ //For all elements in that location
                        String typeOfContent = Types.get(content);
                        if(goalType.containsKey(typeOfContent)){
                            return true;
                        }
                    }
                }
            }
            return false;
        }

        //if there is a goal content in that location then the move makes sense
        else if(a.getName().contains("deliver-to-location")){
            String location = Constants.get(a.getValueOfParameter(1));
            String box = Constants.get(a.getValueOfParameter(2));
            String contentInBox = checkerBoxContent.boxContent.get(box);

            //if the box has a content and its a goal content then the deliver to location move is wrong
            if(contentInBox!=null){
                boolean isAGoalContent = goalType.containsKey(Types.get(contentInBox));
                if(isAGoalContent) return false;
            }
            if(checkerBoxContent.contentLocation.containsKey(location)){
                    List<String> contentInLocation = checkerBoxContent.contentLocation.get(location);
                    for(String content : contentInLocation){ //For all elements in that location
                        String typeOfContent = Types.get(content);
                        if(goalType.containsKey(typeOfContent)){
                            return true;
                        }
                    }
                }
            return false;
        }

        //if the box is empty or the content is not the one requested than the action is wrong
        else if(a.getName().contains("deliver-to-workstation")){
            String workstation = Constants.get(a.getValueOfParameter(1));
            String box = Constants.get(a.getValueOfParameter(3));
            String contentInsideBox = checkerBoxContent.boxContent.get(box);
            if(contentInsideBox==null) return false;

            if(goalWS.contains(workstation))
             if(goal.get(workstation).contains(Types.get(contentInsideBox)))
                return true;
            return false;
        }

        //If the result is true, that means there is a box with a goal content in another place 
        else if(a.getName().contains("pick-up-from-workstation")){
            return !checkerBoxContent.checkPresence(goalType.keySet()); 
        }

        else if(a.getName().contains("pick-up-from-location")){
            String box = Constants.get(a.getValueOfParameter(1));
            String loc = Constants.get(a.getValueOfParameter(2));
            String contentInsideBox = checkerBoxContent.boxContent.get(box);
            if(contentInsideBox!=null){ //if the box is full and not contain a goal content than the move is wrong
                boolean isAGoalContent = goalType.containsKey(Types.get(contentInsideBox));
                if(!isAGoalContent)
                    return false;
            }
            else{ //if the box is empty and there is a goal content in that loction the pickup is wrong
                if(checkerBoxContent.contentLocation.containsKey(loc)){
                    List<String> contentInLocation = checkerBoxContent.contentLocation.get(loc);
                    for(String content : contentInLocation)
                        if(goalType.containsKey(Types.get(content)))
                            return false;
                }
            }
        }
        
        return true;
    }

    /**
     * Returns if a specified problem is supported by the planner. Just ADL problem can be solved by this planner.
     *
     * @param problem the problem to test.
     * @return <code>true</code> if the problem is supported <code>false</code> otherwise.
     */
    @Override
    public boolean isSupported(Problem problem) {
        return (problem.getRequirements().contains(RequireKey.ACTION_COSTS)
            || problem.getRequirements().contains(RequireKey.CONSTRAINTS)
            || problem.getRequirements().contains(RequireKey.CONTINOUS_EFFECTS)
            || problem.getRequirements().contains(RequireKey.DERIVED_PREDICATES)
            || problem.getRequirements().contains(RequireKey.DURATIVE_ACTIONS)
            || problem.getRequirements().contains(RequireKey.DURATION_INEQUALITIES)
            || problem.getRequirements().contains(RequireKey.FLUENTS)
            || problem.getRequirements().contains(RequireKey.GOAL_UTILITIES)
            || problem.getRequirements().contains(RequireKey.METHOD_CONSTRAINTS)
            || problem.getRequirements().contains(RequireKey.NUMERIC_FLUENTS)
            || problem.getRequirements().contains(RequireKey.OBJECT_FLUENTS)
            || problem.getRequirements().contains(RequireKey.PREFERENCES)
            || problem.getRequirements().contains(RequireKey.TIMED_INITIAL_LITERALS)
            || problem.getRequirements().contains(RequireKey.HIERARCHY))
            ? false : true;
    }

        /**
     * Extracts a search from a specified node.
     *
     * @param node    the node.
     * @param problem the problem.
     * @return the search extracted from the specified node.
     */
    private Plan extractPlan(final Node node, final Problem problem) {
        Node n = node;
        final Plan plan = new SequentialPlan();
        while (n.getAction() != -1) {
            final Action a = problem.getActions().get(n.getAction());
            plan.add(0, a);
            n = n.getParent();
        }
        return plan;
    }


    /**
     * The main method of the <code>ASP</code> planner.
     *
     * @param args the arguments of the command line.
     */
    public static void main(String[] args) {
        try {
            final ASP planner = new ASP();
            CommandLine cmd = new CommandLine(planner);
            cmd.execute(args);
        } catch (IllegalArgumentException e) {
            LOGGER.fatal(e.getMessage());
        }
    }
}