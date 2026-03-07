package frc.robot.auto.states;

import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.Drivetrain;

public class AutoStateDrive extends AbstractAutoState {
    private double travelAngle;
    private double robotEndAngle;
    private Drivetrain drivetrain;
    private double distance;
    private double velocity = 1; // meters / second
    private double turnVelocity = Math.PI / 2; // rad / second

    private double targetHeading = 0;

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
        targetHeading = drivetrain.getHeading() + robotEndAngle;

        // estimated time for driving: 
        double estimatedTime = this.distance / this.velocity; 
        turnVelocity = Math.min(this.robotEndAngle / estimatedTime, DriveConstants.MAX_ANGULAR_SPEED);
    }

    @Override
    public void action(double periodSeconds) {
        // TODO: the rotation rate limelight.getTV() == 1is not correct, we'd probably
        // rotate at a fixed
        // speed, and we should
        // either include this check in atDistance or make another check which is
        // atAngle.
        if (atAngle(this)) {
            drivetrain.drive(velocity * Math.sin(travelAngle), velocity * Math.cos(travelAngle),
                    0, false,
                    periodSeconds);
        }  else if (atDistance(this)) {
            drivetrain.drive(0, 0,
                    turnVelocity, false,
                    periodSeconds);
        } else {
            drivetrain.drive(velocity * Math.sin(travelAngle), velocity * Math.cos(travelAngle),
                    turnVelocity, false,
                    periodSeconds);
        }
    }

    public boolean atAngle(AbstractAutoState state) {
        return Math.abs(drivetrain.getHeading() - targetHeading) < Constants.DriveConstants.ALIGN_TOLERANCE;
    }

    public boolean atDistance(AbstractAutoState state) {
        return Math.abs(this.distance
                - drivetrain.getDrivePositionMetersFrontLeft()) < Constants.DriveConstants.DISTANCE_TOLERANCE;
    }

}