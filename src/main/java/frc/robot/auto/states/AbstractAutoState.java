package frc.robot.auto.states;

import java.util.ArrayList;

public abstract class AbstractAutoState {

    private ArrayList<AutoTransition> transitions = new ArrayList<>();

    public AbstractAutoState getNextState() {
        for (int i = 0; i < transitions.size(); i++) {
            if (transitions.get(i).shouldTransition()) {
                return transitions.get(i).getTargetState();
            }
        }
        return null;
    }

    public void addTransition(AutoTransition transition) {
        transitions.add(transition);
    }

    public void onStateEntry(double periodSeconds) {
    }

    public void onStateExit(double periodSeconds) {

    }

    abstract public void action(double periodSeconds);
}
