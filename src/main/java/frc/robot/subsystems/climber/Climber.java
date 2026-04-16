package frc.robot.subsystems.climber;

import frc.robot.Configs;
import frc.robot.Constants.ClimberConstants;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Climber {
    // private final SparkMax climberMotor;
    // private final RelativeEncoder climberEncoder;

    public Climber() {
        // climberMotor = new SparkMax(ClimberConstants.CLIMBER_ID,
        // MotorType.kBrushless);
        // climberEncoder = climberMotor.getEncoder();

        // climberMotor.configure(
        // Configs.Climber.climberConfig,
        // ResetMode.kResetSafeParameters,
        // PersistMode.kPersistParameters);
    }

    public void robotInit() {
        // climberEncoder.setPosition(0);
    }

    public void stageInit() {
        stopClimber();
    }

    public void periodic() {
        updateDashboard();
    }

    public void extendClimber() {
        climb(ClimberConstants.MAX_HEIGHT_ROTATIONS);
    }

    public void flexClimber() {
        climb(ClimberConstants.CLIMB_HEIGHT_ROTATIONS);
    }

    public void retractClimber() {
        climb(ClimberConstants.REST_HEIGHT_ROTATIONS);
    }

    public void stopClimber() {
        // climberMotor.set(0);
    }

    public void driveDownForReset() {
        // climberMotor.set(-0.3);
    }

    public void driveUpForReset() {
        // climberMotor.set(0.3);
    }

    public void climb(double targetHeightTicks) {
        // if (getPosition() < targetHeightTicks -
        // ClimberConstants.CLIMB_ROTATIONS_TOLERANCE) {
        // climberMotor.set(0.4);
        // } else if (getPosition() > targetHeightTicks +
        // ClimberConstants.CLIMB_ROTATIONS_TOLERANCE) {
        // climberMotor.set(-1);
        // } else {
        // climberMotor.set(0);
        // }
    }

    public double getPosition() {
        return 0;
        // return climberEncoder.getPosition();
    }

    public void lockClimbMotor() {
        // Configs.Climber.climberConfig.idleMode(SparkBaseConfig.IdleMode.kBrake);
        // climberMotor.configure(Configs.Climber.climberConfig,
        // ResetMode.kNoResetSafeParameters,
        // PersistMode.kNoPersistParameters);
    }

    public void unlockClimbMotor() {
        // Configs.Climber.climberConfig.idleMode(SparkBaseConfig.IdleMode.kCoast);
        // climberMotor.configure(Configs.Climber.climberConfig,
        // ResetMode.kNoResetSafeParameters,
        // PersistMode.kNoPersistParameters);
    }

    private void updateDashboard() {
        // SmartDashboard.putNumber("ClimberEncoder", climberEncoder.getPosition());
    }
}
