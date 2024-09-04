package fr.uga.pddl4j.examples.asp;

import fr.uga.pddl4j.heuristics.state.RelaxedGraphHeuristic;
import fr.uga.pddl4j.planners.statespace.search.Node;
import fr.uga.pddl4j.problem.Problem;
import fr.uga.pddl4j.problem.State;
import fr.uga.pddl4j.problem.operator.Action;
import fr.uga.pddl4j.problem.operator.Condition;
import fr.uga.pddl4j.problem.Fluent;
import java.util.HashMap;
import java.util.List;
import java.util.*;
import java.io.*;
import fr.uga.pddl4j.util.BitVector;

public final class FortugnoGrecoPrestaHeuristic extends RelaxedGraphHeuristic {
    
    public FortugnoGrecoPrestaHeuristic(Problem problem) 
    {
        super(problem);
    }

    @Override
    public int estimate(State state, Condition goal) {
        super.setGoal(goal);
        super.expandRelaxedPlanningGraph(state);
        return super.isGoalReachable() ? 1 : Integer.MAX_VALUE;
    }

    @Override
    public double estimate(Node node, Condition goal) {
        return this.estimate((State) node, goal);
    }

    public double customEstimate(State state, Condition goal, Action a, double currentHeuristic) {
        super.setGoal(goal);
        super.expandRelaxedPlanningGraph(state);

        BitVector positiveFluents = goal.getPositiveFluents(); // Prende i fluents positivi del goal
        int satisfiedFluents = 0;

        // Conta i fluents positivi del goal che sono soddisfatti dallo stato attuale
        for (int i = positiveFluents.nextSetBit(0); i >= 0; i = positiveFluents.nextSetBit(i + 1)) {
            BitVector fluentCheck = new BitVector();
            fluentCheck.set(i);

            if (state.satisfy(new Condition(fluentCheck, new BitVector()))) {
                satisfiedFluents++;
            }
        }

        BitVector negativeFluents = goal.getNegativeFluents(); // Prende i fluents negativi del goal

        // Conta i fluents negativi del goal che sono soddisfatti dallo stato attuale
        for (int i = negativeFluents.nextSetBit(0); i >= 0; i = negativeFluents.nextSetBit(i + 1)) {
            BitVector fluentCheck = new BitVector();
            fluentCheck.set(i);

            if (!state.satisfy(new Condition(fluentCheck, new BitVector()))) {
                satisfiedFluents++;
            }
        }

        // Calcola la differenza tra il totale dei fluents del goal e quelli soddisfatti dallo stato attuale
        int missingFluents = (positiveFluents.cardinality() + negativeFluents.cardinality()) - satisfiedFluents;

        // Aggiunge una penalità se l'azione è un "move"
        double movePenalty = a.getName().contains("move") ? 1.5 : 0.0;

        // Restituisce il numero di fluents non soddisfatti con la penalità aggiuntiva per le mosse
        return missingFluents + movePenalty + (0.1 * currentHeuristic);
    } 



}
