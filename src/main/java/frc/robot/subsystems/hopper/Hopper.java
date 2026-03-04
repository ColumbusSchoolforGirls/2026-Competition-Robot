package frc.robot.subsystems.hopper;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import frc.robot.Configs;
import frc.robot.Constants.HopperConstants;

public class Hopper {

    private final SparkMax hopperMotor = new SparkMax(HopperConstants.HOPPER_ID, MotorType.kBrushless);

    public Hopper() {
        hopperMotor.configure(
                Configs.Hopper.hopperConfig,
                ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);
    }

    public void setHopper(boolean shooterRunning, boolean intakeRunning) {
        if (shooterRunning || intakeRunning) {
            hopperMotor.set(HopperConstants.HOPPER_PERCENTAGE_OUTPUT);
        } else {
            hopperMotor.set(0);
        }
    }
}
