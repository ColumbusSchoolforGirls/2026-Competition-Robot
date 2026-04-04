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

    private boolean driving;

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

    public void robotInit() {
        deployEncoder.setPosition(0);
        stop();
    }

    public void stageInit() {
        stop();
    }

    public void periodic() {
        updateDashboard();
    }

    public void updateDashboard() {
        SmartDashboard.putBoolean("IntakeRetracted", isRetracted());
        SmartDashboard.putNumber("RollerMotorPercent", rollerMotor.getMotorOutputPercent());
        SmartDashboard.putNumber("Intake Rotations", deployEncoder.getPosition());
        SmartDashboard.putBoolean("Intake Running", driving);

    }

    public void deploy() {
        if (!isDeployed()) {
            deployMotor.set(IntakeConstants.DEPLOY_PERCENTAGE_OUTPUT);
            driving = true;
        } else {
            deployMotor.set(0);
            driving = false;
        }
    }

    public void retract() {
        if (!isRetracted()) {
            deployMotor.set(IntakeConstants.RETRACT_PERCENTAGE_OUTPUT);
            driving = true;
        } else {
            deployMotor.set(0);
            driving = false;
        }
    }

    public void stop() {
        deployMotor.set(0);
        rollerMotor.set(VictorSPXControlMode.PercentOutput, 0);
    }

    public void stopRoller() {
        rollerMotor.set(VictorSPXControlMode.PercentOutput, 0);
    }

    public void runRoller(boolean runRoller) {
        if (runRoller) {
            rollerMotor.set(VictorSPXControlMode.PercentOutput, IntakeConstants.ROLLER_PERCENTAGE_OUTPUT);
        } else {
            rollerMotor.set(VictorSPXControlMode.PercentOutput, 0);
        }
    }

    public void emptyRoller(boolean emptyRoller) {
        if (emptyRoller) {
            rollerMotor.set(VictorSPXControlMode.PercentOutput, -IntakeConstants.ROLLER_PERCENTAGE_OUTPUT);
        } else {
            rollerMotor.set(VictorSPXControlMode.PercentOutput, 0);
        }
    }

    private boolean isDeployed() {
        return deployEncoder.getPosition() >= IntakeConstants.DEPLOYED_ROTATIONS_DISTANCE;
    }

    private boolean isRetracted() {
        boolean isRetracted = !retractedLimitSwitch.get();
        if (isRetracted) {
            deployEncoder.setPosition(0);
            return true;
        }
        return false;
    }
}
