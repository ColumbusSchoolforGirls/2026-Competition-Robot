package frc.robot.auto.states;

import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;

public class AutoStateDrive extends AbstractAutoState {
    private double travelAngle;
    private double robotEndAngle;
    private Drivetrain drivetrain;
    private double distance;
    private double velocity = 1;

    public AutoStateDrive(double distance, double travelAngle, double robotEndAngle, Drivetrain drivetrain,
            double velocity) {
        this.distance = distance;
        this.travelAngle = travelAngle;
        this.robotEndAngle = robotEndAngle;
        this.drivetrain = drivetrain;
        this.velocity = velocity;
    }

    @Override
    public void startState() {
        drivetrain.resetDistance();
    }

    @Override
    public void action(double periodSeconds) {
        // TODO: the rotation rate limelight.getTV() == 1is not correct, we'd probably
        // rotate at a fixed
        // speed, and we should
        // either include this check in atDistance or make another check which is
        // atAngle.
        drivetrain.drive(velocity * Math.sin(travelAngle), velocity * Math.cos(travelAngle),
                (robotEndAngle - drivetrain.getHeading()), false,
                periodSeconds);
    }

    public boolean atDistance(AbstractAutoState state) {
        return this.distance
                - drivetrain.getDrivePositionMetersFrontLeft() < Constants.DriveConstants.DISTANCE_TOLERANCE;
    }

}