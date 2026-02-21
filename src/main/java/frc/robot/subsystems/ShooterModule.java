package frc.robot.subsystems;

import frc.robot.Configs;

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
        leadMotor = new SparkMax(leadMotorID, MotorType.kBrushless);
        followerMotor = new SparkMax(followerMotorID, MotorType.kBrushless);
        feedMotor = new SparkMax(feedMotorID, MotorType.kBrushless);

        Configs.Shooter.shooterFollowerConfig
                .apply(Configs.Shooter.shooterConfig)
                .follow(leadMotorID, true);

        leadMotor.configure(
                Configs.Shooter.shooterConfig,
                ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);
        followerMotor.configure(
                Configs.Shooter.shooterFollowerConfig,
                ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);
        feedMotor.configure(
                Configs.Shooter.feederConfig,
                ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);

        leadPidController = leadMotor.getClosedLoopController();
    }

    public RelativeEncoder getEncoder() {
        return leadMotor.getEncoder();
    }

    public double getShooterRPM() {
        return leadMotor.getEncoder().getVelocity();
    }

    public void setShooterRPM(double speed) {
        leadPidController.setSetpoint(speed, ControlType.kVelocity);
    }

    public void setFeeder(double percentageOutput) {
        feedMotor.set(percentageOutput);
    }
}
