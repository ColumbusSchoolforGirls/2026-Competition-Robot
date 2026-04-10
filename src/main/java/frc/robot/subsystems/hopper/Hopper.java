package frc.robot.subsystems.hopper;

import frc.robot.Constants.HopperConstants;

import com.ctre.phoenix.motorcontrol.VictorSPXControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

public class Hopper {
    private final VictorSPX hopperMotor;

    private boolean runHopper;
    private boolean expelSystem;

    public Hopper() {
        hopperMotor = new VictorSPX(HopperConstants.HOPPER_ID);
        runHopper = false;
        expelSystem = false;
    }

    public void setHopper() {
        hopperMotor.set(VictorSPXControlMode.PercentOutput, determineHopperOutput());
    }

    public void runHopper(boolean runHopper) {
        this.runHopper = runHopper;
    }

    public void expelHopper(boolean expelSystem) {
        this.expelSystem = expelSystem;
    }

    private double determineHopperOutput() {
        if (expelSystem) {
            return HopperConstants.HOPPER_PERCENTAGE_EXPEL_OUTPUT;
        } else if (runHopper) {
            return HopperConstants.HOPPER_PERCENTAGE_OUTPUT;
        }
        return 0;
    }
}
