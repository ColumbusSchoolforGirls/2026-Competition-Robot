package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

public class Shooter {

    private final SparkMax shooterMotor;

    public Shooter(int shooterMotorID) { // TODO: Make constant when implemented(?)
        shooterMotor = new SparkMax(shooterMotorID, MotorType.kBrushless);
    }

    public void setShooterSpeed(double speed) {
        shooterMotor.set(speed);
    }

    public void stopShooter() {
        shooterMotor.set(0);
    }

    // public void update() { //TODO: Move to joystick controls when implemented
    // if (AUX.getAButton()) {
    // setShooterSpeed(ShooterConstants.SHOOTER_SPEED);
    // } else {
    // stopShooter();
    // }
    // }
}
