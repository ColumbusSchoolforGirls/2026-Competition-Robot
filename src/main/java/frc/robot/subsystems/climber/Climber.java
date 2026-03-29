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
    private final SparkMax climberMotor;

    private final RelativeEncoder climberEncoder;

    public Climber() {
        climberMotor = new SparkMax(ClimberConstants.CLIMBER_ID, MotorType.kBrushless);

        climberEncoder = climberMotor.getEncoder();

        climberMotor.configure(
                Configs.Climber.climberConfig,
                ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);
    }

    public void init() {
        climberEncoder.setPosition(0);
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
        climberMotor.set(0);
    }

    public void driveDown() {
        climberMotor.set(-0.1);
    }

    public void climb(double targetHeightTicks) {
        if (getPosition() < targetHeightTicks - ClimberConstants.CLIMB_ROTATIONS_TOLERANCE) {
            climberMotor.set(0.2);
        } else if (getPosition() > targetHeightTicks + ClimberConstants.CLIMB_ROTATIONS_TOLERANCE) {
            climberMotor.set(-0.2);
        } else {
            climberMotor.set(0);
        }
    }

    public double getPosition() {
        return climberEncoder.getPosition();
    }

    public void lockClimbMotor() {
        Configs.Climber.climberConfig.idleMode(SparkBaseConfig.IdleMode.kBrake);
        climberMotor.configure(Configs.Climber.climberConfig, ResetMode.kNoResetSafeParameters,
                PersistMode.kNoPersistParameters);
    }

    public void unlockClimbMotor() {
        Configs.Climber.climberConfig.idleMode(SparkBaseConfig.IdleMode.kCoast);
        climberMotor.configure(Configs.Climber.climberConfig, ResetMode.kNoResetSafeParameters,
                PersistMode.kNoPersistParameters);
    }

    public void updateDashboard() {
        SmartDashboard.putNumber("ClimberEncoder", climberEncoder.getPosition());
    }
}
