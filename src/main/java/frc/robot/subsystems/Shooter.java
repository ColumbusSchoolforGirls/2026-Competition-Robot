package frc.robot.subsystems;

import frc.robot.Configs.ShooterSystem;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;

public class Shooter {

    private final SparkMax leadMotor;
    private final SparkMax followerMotor;

    private SparkClosedLoopController leadPidController;

    public Shooter(int leadMotorID, int followerMotorID) {
        this.leadMotor = new SparkMax(leadMotorID, MotorType.kBrushless);
        this.followerMotor = new SparkMax(followerMotorID, MotorType.kBrushless);

        this.leadMotor.configure(
                ShooterSystem.shooterConfig,
                ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);
        this.followerMotor.configure(
                ShooterSystem.shooterFollowerConfig,
                ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);
        this.leadPidController = this.leadMotor.getClosedLoopController();
    }

    public RelativeEncoder getEncoder() {
        return this.leadMotor.getEncoder();
    }

    public void setShooterSpeed(double speed) {
        this.leadPidController.setSetpoint(speed, ControlType.kDutyCycle);
    }

    public void stopShooter() {
        this.leadMotor.set(0);
    }
}
