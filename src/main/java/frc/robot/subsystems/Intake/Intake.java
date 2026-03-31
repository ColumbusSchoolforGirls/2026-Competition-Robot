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

    public void init() {
        deployEncoder.setPosition(0);
        state = IntakeState.IN;
        runRoller = false;
        setMotors();
    }

    public void periodic() {
        updateDashboard();
    }

    public void updateDashboard() {
        SmartDashboard.putString("IntakeState", state.toString());
        SmartDashboard.putBoolean("IntakeRetracted", isRetracted());
        SmartDashboard.putNumber("RollerMotorPercent", rollerMotor.getMotorOutputPercent());
        SmartDashboard.putNumber("Intake Rotations", deployEncoder.getPosition());
    }

    public void deploy() {
        if (!isDeployed()) {
            state = IntakeState.DEPLOYING;
            setMotors();
        } else {
            state = IntakeState.OUT;
            setMotors();
        }
    }

    public void retract() {
        if (!isRetracted()) {
            state = IntakeState.RETRACTING;
            setMotors();
        } else {
            state = IntakeState.IN;
            setMotors();
        }
    }

    public void stop() {
        deployMotor.set(0);
        rollerMotor.set(VictorSPXControlMode.PercentOutput, 0);
    }

    public void setRoller(boolean runRoller) {
        this.runRoller = runRoller;
        setMotors();
        updateDashboard();
    }

    private boolean isRetracted() {
        if (retractedLimitSwitch.get()) {
            deployEncoder.setPosition(0);
            return true;
        }
        return false;
    }

    private boolean isDeployed() {
        return deployEncoder.getPosition() >= IntakeConstants.DEPLOYED_ROTATIONS_DISTANCE;
    }

    private void setMotors() {
        deployMotor.set(determineDeployPercentageOutput());
        rollerMotor.set(VictorSPXControlMode.PercentOutput, determineRollerPercentageOutput());
    }

    private double determineDeployPercentageOutput() {
        if (state == IntakeState.DEPLOYING) {
            return IntakeConstants.DEPLOY_PERCENTAGE_OUTPUT;
        } else if (state == IntakeState.RETRACTING) {
            return IntakeConstants.RETRACT_PERCENTAGE_OUTPUT;
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
