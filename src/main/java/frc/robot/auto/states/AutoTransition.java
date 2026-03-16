package frc.robot.auto.states;

import java.util.function.Predicate;

public class AutoTransition {
    private AbstractAutoState targetState;
    private Predicate<AbstractAutoState> shouldTransitionFunction;

    public AutoTransition(AbstractAutoState targetState, Predicate<AbstractAutoState> shouldTransitionFunction) {
        this.targetState = targetState;
        this.shouldTransitionFunction = shouldTransitionFunction;
    }

    public boolean shouldTransition() {
        return shouldTransitionFunction.test(targetState);
    }

    public AbstractAutoState getTargetState() {
        return targetState;
    }

}