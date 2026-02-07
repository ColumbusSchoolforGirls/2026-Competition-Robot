package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import static frc.robot.Constants.ControllerConstants.AUX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {

    private final SparkMax ShooterMotor;

    public Shooter(int shooterMotorID) {
        ShooterMotor = new SparkMax(shooterMotorID, MotorType.kBrushless);
    }

    public void setShooterSpeed(double speed) {
        ShooterMotor.set(speed);
    }

    public void stopShooter() {
        ShooterMotor.set(0);
    }

    public void update() {
        if (AUX.getAButton()) {
            setShooterSpeed(0.75);
            System.out.println("test");
        } else {
            stopShooter();
        }
    }

}
