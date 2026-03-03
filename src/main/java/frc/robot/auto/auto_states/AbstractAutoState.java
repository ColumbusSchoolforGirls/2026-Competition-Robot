package frc.robot.auto.auto_states;

import java.util.ArrayList;
import java.util.function.Predicate;

public abstract class AbstractAutoState {

    private ArrayList<Predicate<AbstractAutoState>> transitions = new ArrayList<>();

    public boolean shouldTransition() {
        for (int i = 0; i < transitions.size(); i++) {
            if (transitions.get(i).test(this)) {
                return true;
            }
        }
        return false;
    }

    public void addTransition(Predicate<AbstractAutoState> predicate) {
        transitions.add(predicate);
    }

    abstract public void action(double periodSeconds);
}
