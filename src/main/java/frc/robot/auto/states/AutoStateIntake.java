package frc.robot.auto.states;

import frc.robot.subsystems.intake.Intake;

public class AutoStateIntake extends AbstractAutoState {
    Intake intake;
    boolean runRollers;

    public AutoStateIntake(Intake intake, boolean runRollers) {
        this.intake = intake;
        this.runRollers = runRollers;
    }

    @Override
    public void onStateEntry(double periodSeconds) {
        intake.runRollers(runRollers);
        intake.setIntakeRollers();
    }

    @Override
    public void action(double periodSeconds) {
        // Does nothing, this is a stopping state
    }

    @Override
    public void onStateExit(double periodSeconds) {
    }
}
