package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import frc.robot.Constants.ShooterConstants;

public class ShooterSystem {

    public enum ShooterAction {
        STOP, REV, SHOOT
    }

    ShooterAction state = ShooterAction.STOP;
    String shooterState = "STOP";

    private final Shooter leftShooter = new Shooter(
            ShooterConstants.LEFT_LEAD_ID,
            ShooterConstants.LEFT_FOLLOWER_ID);
    private final Shooter rightShooter = new Shooter(
            ShooterConstants.RIGHT_LEAD_ID,
            ShooterConstants.RIGHT_FOLLOWER_ID);
    private final SparkMax leftFeederMotor = new SparkMax(ShooterConstants.LEFT_FEEDER_ID, MotorType.kBrushless);
    private final SparkMax rightFeederMotor = new SparkMax(ShooterConstants.RIGHT_FEEDER_ID, MotorType.kBrushless);

    private RelativeEncoder leftShooterEncoder = leftShooter.getEncoder();
    private RelativeEncoder rightShooterEncoder = rightShooter.getEncoder();
    private RelativeEncoder leftFeederEncoder = leftFeederMotor.getEncoder();
    private RelativeEncoder rightFeederEncoder = rightFeederMotor.getEncoder();

    public ShooterSystem() {
        this.state = ShooterAction.STOP;
    }

    public void update() {
        shoot();
    }

    public void shoot() {
        // TODO: Implement with JoystickControls class
        // shooterState = this.state.toString();

        // if (AUX.getYButtonPressed()) {
        // this.state = ShooterAction.STOP;
        // }

        // switch (this.state) {
        // case STOP: {
        // setMotors();
        // if (AUX.getRightBumperButtonPressed()) {
        // this.state = ShooterAction.REV;
        // }
        // }
        // case REV: {
        // setMotors();
        // if (AUX.getRightTriggerAxis() > 0.5) {
        // this.state = ShooterAction.SHOOT;
        // }
        // }
        // case SHOOT: {
        // setMotors();
        // }
        // default: {
        // setMotors();
        // }
        // }
    }

    // private void setMotors() {
    // feederMotor.set(getFeederSpeed());
    // shooterPidController.setReference(getShooterSpeed(),
    // SparkMax.ControlType.kVelocity);
    // }

    private double getShooterSpeed() {
        if (this.state == ShooterAction.SHOOT || this.state == ShooterAction.REV) {
            return ShooterConstants.SHOOT_SPEED;
        }
        return 0.0;
    }

    private double getFeederSpeed() {
        if (this.state == ShooterAction.SHOOT) {
            return ShooterConstants.SHOOT_SPEED;
        }
        return 0.0;
    }
}