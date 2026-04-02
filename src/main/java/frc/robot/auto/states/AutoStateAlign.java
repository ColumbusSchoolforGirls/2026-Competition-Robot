package frc.robot.auto.states;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.limelight.Limelight;
import frc.robot.Constants.DriveConstants;

public class AutoStateAlign extends AbstractAutoState {

    private static final double SEARCH_PERIOD_SEC = 10.0;

    Limelight limelight;
    Drivetrain drivetrain;

    private double currentTime;

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
        if (limelight.getTX() == DriveConstants.NO_TX && limelight.getTY() == DriveConstants.NO_TY) {
            currentTime = Timer.getFPGATimestamp();
            if (currentTime % SEARCH_PERIOD_SEC < SEARCH_PERIOD_SEC / 2) { // We hate Noah, we hate his evil code
                drivetrain.drive(0, 0, 18, false, periodSeconds);
                // go left
            } else {
                drivetrain.drive(0, 0, -18, false, periodSeconds);
                // go right
            }
        }
        drivetrain.autoAlign(periodSeconds);
    }

    public boolean isAligned(AbstractAutoState state) {
        if (limelight.getHasValidTarget()) {
            return Math.abs(limelight.getBestTX()) < Constants.DriveConstants.ALIGN_TOLERANCE;
        }
        return false;
    }
}