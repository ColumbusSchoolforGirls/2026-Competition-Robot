package frc.robot.subsystems.Intake;

import frc.robot.Configs;
import frc.robot.Constants.IntakeConstants;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.motorcontrol.PWMMotorController;
import edu.wpi.first.wpilibj.motorcontrol.Talon;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

public class Intake {
    private final SparkMax deployMotor = new SparkMax(IntakeConstants.DEPLOY_ID, MotorType.kBrushless);
    private final PWMMotorController rollerMotor = new Talon(IntakeConstants.ROLLER_CHANNEL); // TODO: Update with
                                                                                              // specific motor
                                                                                              // controller type

    private final RelativeEncoder deployEncoder = deployMotor.getEncoder();

    private static DigitalInput retractedLimitSwitch = new DigitalInput(IntakeConstants.RETRACTED_LIMIT_SWITCH_CHANNEL);

    private IntakeState state;
    private boolean runRoller;

    public Intake() {
        deployMotor.configure(
                Configs.Intake.deployConfig,
                ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);
    }

    public void setIntakeState(IntakeState state, boolean runRoller) {
        this.state = state;
        this.runRoller = runRoller;
        setMotors();
        updateDashboard();
    }

    public boolean isRetracted() {
        if (retractedLimitSwitch.get()) {
            deployEncoder.setPosition(0);
            return true;
        }
        return false;
    }

    public boolean isDeployed() {
        double positionDifference = IntakeConstants.DEPLOYED_TICK_DISTANCE - deployEncoder.getPosition();
        return positionDifference <= 0;
    }

    private void updateDashboard() {
        SmartDashboard.putString("IntakeState", state.toString());
        SmartDashboard.putBoolean("IntakeRetracted", isRetracted());
    }

    private void setMotors() {
        deployMotor.set(determineDeployPercentageOutput());
        rollerMotor.set(determineRollerPercentageOutput());
        hopper.setHopper(false, runHopper());
    }

    private double determineDeployPercentageOutput() {
        if (state == IntakeState.DEPLOYING) {
            if (!isDeployed()) {
                return IntakeConstants.DEPLOY_PERCENTAGE_OUTPUT;
            }
            return 0;
        } else if (state == IntakeState.RETRACTING) {
            if (!isRetracted()) {
                return -IntakeConstants.DEPLOY_PERCENTAGE_OUTPUT;
            }
            return 0;
        }
        return 0;
    }

    private double determineRollerPercentageOutput() {
        if (runRoller) {
            return IntakeConstants.ROLLER_PERCENTAGE_OUTPUT;
        }
        return 0;
    }
}
