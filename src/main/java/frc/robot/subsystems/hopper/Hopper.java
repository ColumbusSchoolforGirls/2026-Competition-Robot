package frc.robot.subsystems.hopper;

import frc.robot.Constants.HopperConstants;

import edu.wpi.first.wpilibj.motorcontrol.PWMMotorController;
import edu.wpi.first.wpilibj.motorcontrol.Talon;

public class Hopper {

    private final PWMMotorController hopperMotor = new Talon(HopperConstants.HOPPER_ID);

    public Hopper() {}

    public void runHopper(boolean running) {
        if (running) {
            hopperMotor.set(HopperConstants.HOPPER_PERCENTAGE_OUTPUT);
        } else {
            hopperMotor.set(0);
        }
    }
}
