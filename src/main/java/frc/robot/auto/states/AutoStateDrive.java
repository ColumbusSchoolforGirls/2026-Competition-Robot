package frc.robot.auto.states;

import frc.robot.Constants;
import frc.robot.subsystems.drivetrain.Drivetrain;

public class AutoStateDrive extends AbstractAutoState {
    private double travelAngleRad;
    private double robotEndAngleDegrees;
    private Drivetrain drivetrain;
    private double distance;
    private double velocity = 1; // meters / second
    private double turnVelocity;

    private double targetHeading = 0;

    public AutoStateDrive(double distance, double travelAngleDegrees, double robotEndAngleDegrees,
            Drivetrain drivetrain,
            double velocity) {
        this.distance = distance;
        this.travelAngleRad = travelAngleDegrees * Math.PI / 180.0;
        this.robotEndAngleDegrees = robotEndAngleDegrees;
        this.drivetrain = drivetrain;
        this.velocity = velocity;
    }

    @Override
    public void onStateEntry(double periodSeconds) {
        drivetrain.resetDistance();
        targetHeading = drivetrain.getRotation2d().getDegrees() + robotEndAngleDegrees;

        if (robotEndAngleDegrees != 0.0) {
            turnVelocity = 1.0;
        }
    }

    @Override
    public void onStateExit(double periodSeconds) {
        drivetrain.drive(0, 0, 0, false, periodSeconds);
    }

    @Override
    public void action(double periodSeconds) {
        // TODO: the rotation rate limelight.getTV() == 1 is not correct, we'd probably
        // rotate at a fixed
        // speed, and we should
        // either include this check in atDistance or make another check which is
        // atAngle.

        System.out.printf("Distance: %.1f, Pos: %.1f\n", this.distance,
                drivetrain.getDrivePositionMeters());

        if (atDistance(this)) {
            drivetrain.drive(0, 0,
                    turnVelocity, false,
                    periodSeconds);
        } else if (atAngle(this)) {
            drivetrain.drive(velocity * Math.cos(travelAngleRad),
                    velocity * Math.sin(travelAngleRad),
                    0, false,
                    periodSeconds);
        } else {
            drivetrain.drive(velocity * Math.cos(travelAngleRad),
                    velocity * Math.sin(travelAngleRad),
                    turnVelocity, false,
                    periodSeconds);
        }
    }

    public boolean atAngle(AbstractAutoState state) {
        return Math.abs(
                drivetrain.getRotation2d().getDegrees() - targetHeading) < Constants.DriveConstants.ALIGN_TOLERANCE;
    }

    public boolean atDistance(AbstractAutoState state) {
        return (Math.abs(this.distance)
                - Math.abs(drivetrain.getDrivePositionMeters())) < Constants.DriveConstants.DISTANCE_TOLERANCE;
    }

}