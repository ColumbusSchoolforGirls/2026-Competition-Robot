package frc.robot.subsystems.Intake;

import frc.robot.Configs;
import frc.robot.Constants.IntakeConstants;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Intake {
    private final SparkMax deployMotor = new SparkMax(IntakeConstants.DEPLOY_ID, MotorType.kBrushless);
    private final WPI_TalonSRX rollerMotor = new WPI_TalonSRX(IntakeConstants.ROLLER_ID);

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

    // TODO: Implement the isDeployed() and isRetracted() using encoder ticks.
    public boolean isRetracted() {
        return retractedLimitSwitch.get();
    }

    public boolean isDeployed() {
        return deployEncoder.getPosition() >= IntakeConstants.DEPLOYED_TICK_DISTANCE;
    }

    // TODO: Move the arm
    private void deployIntake() {
        
    }

    private void resetIntakeEncoder() {
        if (isRetracted()) {
            deployEncoder.setPosition(0);
        }
    }

    private void updateDashboard() {
        SmartDashboard.putString("IntakeState", state.toString());
        SmartDashboard.putBoolean("IntakeRetracted", isRetracted());
    }

    private void setMotors() {
        deployMotor.set(determineDeployPercentageOutput());
        rollerMotor.set(determineRollerPercentageOutput());
    }

    private double determineDeployPercentageOutput() {
        if (state == IntakeState.DEPLOYING) {
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
