package frc.robot.auto.states;

import frc.robot.subsystems.drivetrain.Drivetrain;

public class AutoStateStop extends AbstractAutoState {
    Drivetrain drivetrain;

    public AutoStateStop(Drivetrain drivetrain) {
        this.drivetrain = drivetrain;
    }

    @Override
    public void action(double periodSeconds) {
        // Does nothing, this is a stopping state
        drivetrain.drive(0, 0, 0, false, periodSeconds);
    }
}
