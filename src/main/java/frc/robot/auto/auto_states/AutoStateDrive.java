package frc.robot.auto.auto_states;

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
    public void action(double periodSeconds) {
        drivetrain.drive(velocity * Math.sin(travelAngle), velocity * Math.cos(travelAngle),
                (robotEndAngle - drivetrain.getHeading()), false,
                periodSeconds);
    }

    public boolean atDistance(AbstractAutoState state) {
        return this.distance
                - drivetrain.frontLeft.getDrivePositionMeters() < Constants.DriveConstants.DISTANCE_TOLERANCE;
    }

}