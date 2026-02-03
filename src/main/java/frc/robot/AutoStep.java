package frc.robot;

public class AutoStep {
    AutoAction action;
    double value;

    public AutoStep(AutoAction action) {
        this.action = action;
    }

    public AutoStep(AutoAction action, double value) {
        this.action = action;
        this.value = value;
    }
    
    public AutoAction getAction() {
        return action;
    }

    public double getValue() {
        return value;
    }

    @Override
    public String toString() {
        return String.format("AutoAction(%s)",action.name());
    }
}
