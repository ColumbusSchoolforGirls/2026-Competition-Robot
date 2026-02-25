package frc.robot.Intake;

import frc.robot.Configs;
import frc.robot.Constants.IntakeConstants;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Intake {
    private final SparkMax deployMotor = new SparkMax(IntakeConstants.DEPLOY_ID, MotorType.kBrushless);
    private final WPI_TalonSRX rollerMotor = new WPI_TalonSRX(IntakeConstants.ROLLER_ID);

    private static DigitalInput deployedLimitSwitch = new DigitalInput(IntakeConstants.DEPLOYED_LIMIT_SWITCH_CHANNEL);
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

    // TODO: Implement the isDeployed() and isRetracted() using encoder ticks.
    public boolean isDeployed() {
        return deployedLimitSwitch.get();
    }

    public boolean isRetracted() {
        return retractedLimitSwitch.get();
    }

    private void updateDashboard() {
        SmartDashboard.putString("IntakeState", state.toString());
        SmartDashboard.putBoolean("IntakeDeployed", isDeployed());
    }

    private void setMotors() {
        deployMotor.set(determineDeployPercentageOutput());
        rollerMotor.set(determineRollerPercentageOutput());
    }

    private double determineDeployPercentageOutput() {
        if (state == IntakeState.DEPLOYING && !isDeployed()) {
            return IntakeConstants.DEPLOY_PERCENTAGE_OUTPUT;
        } else if (state == IntakeState.RETRACTING) {
            return -IntakeConstants.DEPLOY_PERCENTAGE_OUTPUT;
        }
        return 0;
    }

    private double determineRollerPercentageOutput() {
        if (state == IntakeState.OUT && runRoller == true) {
            return IntakeConstants.ROLLER_PERCENTAGE_OUTPUT;
        } else {
            return 0.0;
        }
    }
}
