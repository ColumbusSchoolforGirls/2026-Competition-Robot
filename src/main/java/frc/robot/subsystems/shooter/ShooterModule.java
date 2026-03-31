package frc.robot.subsystems.shooter;

import frc.robot.Configs;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.motorcontrol.PWMMotorController;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;

public class ShooterModule {

    private final SparkMax leadMotor;
    private final SparkMax followerMotor;
    private final PWMMotorController feedMotor;
    private final SparkClosedLoopController leadPidController;

    public ShooterModule(int leadMotorID, int followerMotorID, int feedMotorID) {
        leadMotor = new SparkMax(leadMotorID, MotorType.kBrushless);
        followerMotor = new SparkMax(followerMotorID, MotorType.kBrushless);
        feedMotor = new Talon(feedMotorID);

        leadPidController = leadMotor.getClosedLoopController();

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
    }

    public RelativeEncoder getEncoder() {
        return leadMotor.getEncoder();
    }

    public SparkMax getLeadMotor() {
        return leadMotor;
    }

    public double getShooterRPM() {
        return leadMotor.getEncoder().getVelocity();
    }

    public void setShooterRPM(double speed) {
        leadPidController.setSetpoint(speed, ControlType.kVelocity);
    }

    public void cutPower() {
        leadMotor.set(0);
        feedMotor.set(0);
    }

    public void setFeeder(double percentageOutput) {
        feedMotor.set(percentageOutput);
    }
}
