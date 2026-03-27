package frc.robot.subsystems.climber;

import frc.robot.Configs;
import frc.robot.Constants.ClimberConstants;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

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

    public void climb(double targetHeightTicks) {
        if (getPosition() < targetHeightTicks) {
            climberMotor.set(1);
        } else if (getPosition() > targetHeightTicks) {
            climberMotor.set(-1);
        } else {
            climberMotor.set(0);
        }
    }

    public double getPosition() {
        return climberEncoder.getPosition();
    }
}
