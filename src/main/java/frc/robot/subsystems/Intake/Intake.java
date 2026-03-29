package frc.robot.subsystems.intake;

import com.ctre.phoenix.motorcontrol.VictorSPXControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Configs;
import frc.robot.Constants.IntakeConstants;
import frc.robot.JoystickControls;

public class Intake {
    private final SparkMax deployMotor;
    private final VictorSPX rollerMotor;
    private final RelativeEncoder deployEncoder;
    private final DigitalInput retractedLimitSwitch;
    // TODO: check with mech about what the limit switches actually are

    private IntakeState state = IntakeState.IN;
    private boolean runRoller;

    public Intake() {
        deployMotor = new SparkMax(IntakeConstants.DEPLOY_ID, MotorType.kBrushless);
        rollerMotor = new VictorSPX(IntakeConstants.ROLLER_ID);
        deployEncoder = deployMotor.getEncoder();
        retractedLimitSwitch = new DigitalInput(IntakeConstants.RETRACTED_LIMIT_SWITCH_CHANNEL);

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
        // System.out.println(state);
        // if (JoystickControls.AUX.getBButton()) {
        // deployMotor.set(IntakeConstants.RETRACT_PERCENTAGE_OUTPUT);
        // } else if (JoystickControls.AUX.getXButton()) {
        // deployMotor.set(IntakeConstants.DEPLOY_PERCENTAGE_OUTPUT);
        // } else {
        // deployMotor.set(0);
        // }

        // System.out.println("Current" + deployMotor.getOutputCurrent());
    }

    public boolean isRetracted() {
        if (retractedLimitSwitch.get()) {
            deployEncoder.setPosition(0);
            return true;
        }
        return false;
    }

    public boolean isDeployed() {
        double positionDifference = IntakeConstants.DEPLOYED_ROTATIONS_DISTANCE - deployEncoder.getPosition();
        return positionDifference <= 0;
    }

    public void updateDashboard() {
        SmartDashboard.putString("IntakeState", state.toString());
        SmartDashboard.putBoolean("IntakeRetracted", isRetracted());
        SmartDashboard.putNumber("RollerMotorPercent", rollerMotor.getMotorOutputPercent());
    }

    private void setMotors() {
        deployMotor.set(determineDeployPercentageOutput());
        rollerMotor.set(VictorSPXControlMode.PercentOutput, determineRollerPercentageOutput());
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
