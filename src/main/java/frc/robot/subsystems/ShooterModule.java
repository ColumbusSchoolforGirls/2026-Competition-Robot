package frc.robot.subsystems;

import frc.robot.Configs.ShooterSystem;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;

public class ShooterModule {

    private final SparkMax leadMotor;
    private final SparkMax followerMotor;
    private final SparkMax feedMotor;
    private SparkClosedLoopController leadPidController;

    public ShooterModule(int leadMotorID, int followerMotorID, int feedMotorID) {
        this.leadMotor = new SparkMax(leadMotorID, MotorType.kBrushless);
        this.followerMotor = new SparkMax(followerMotorID, MotorType.kBrushless);
        this.feedMotor = new SparkMax(feedMotorID, MotorType.kBrushless);

        ShooterSystem.shooterFollowerConfig
                .apply(ShooterSystem.shooterConfig)
                .follow(leadMotorID, true);

        this.leadMotor.configure(
                ShooterSystem.shooterConfig,
                ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);
        this.followerMotor.configure(
                ShooterSystem.shooterFollowerConfig,
                ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);
        this.feedMotor.configure(
                ShooterSystem.feederConfig,
                ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);

        this.leadPidController = this.leadMotor.getClosedLoopController();
    }

    public RelativeEncoder getEncoder() {
        return this.leadMotor.getEncoder();
    }

    public double getShooterRPM() {
        return this.leadMotor.getEncoder().getVelocity();
    }

    public void setShooterRPM(double speed) {
        this.leadPidController.setSetpoint(speed, ControlType.kVelocity);
    }

    public void setFeed(double percentageOutput) {
        this.feedMotor.set(percentageOutput);
    }
}
