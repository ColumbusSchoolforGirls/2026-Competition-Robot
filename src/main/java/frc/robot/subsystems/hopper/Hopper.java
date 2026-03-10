package frc.robot.subsystems.hopper;

import frc.robot.Constants.HopperConstants;

import com.ctre.phoenix.motorcontrol.VictorSPXControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

public class Hopper {

    private final VictorSPX hopperMotor = new VictorSPX(HopperConstants.HOPPER_ID);

    public Hopper() {}

    public void runHopper(boolean running) {
        if (running) {
            hopperMotor.set(VictorSPXControlMode.PercentOutput, HopperConstants.HOPPER_PERCENTAGE_OUTPUT);
        } else {
            hopperMotor.set(VictorSPXControlMode.PercentOutput,0);
        }
    }
}
