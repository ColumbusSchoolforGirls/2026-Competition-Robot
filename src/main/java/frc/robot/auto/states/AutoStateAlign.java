package frc.robot.auto.states;

import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Limelight;

public class AutoStateAlign extends AbstractAutoState {
    Limelight limelight;
    Drivetrain drivetrain;

    public AutoStateAlign(Limelight limelight, Drivetrain drivetrain) {
        this.limelight = limelight;
        this.drivetrain = drivetrain;
    }

    @Override
    public void onStateExit(double periodSeconds) {
        drivetrain.drive(0, 0, 0, false, periodSeconds);
    }

    @Override
    public void action(double periodSeconds) {
        // TODO: Make a funtion where if no valid target maybe pan from left to right
        drivetrain.autoAlign(periodSeconds);
    }

    public boolean isAligned(AbstractAutoState state) {
        if (limelight.getHasValidTarget()) {
            return Math.abs(limelight.getBestTX()) < Constants.DriveConstants.ALIGN_TOLERANCE;
        }
        return false;
    }
}