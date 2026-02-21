package frc.robot.subsystems;

import frc.robot.Configs;
import frc.robot.Constants.IntakeConstants;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.revrobotics.spark.SparkMax;

public class Intake {

    public enum IntakeState {
        IN, DEPLOY, OUT, RETRACT
    }

    private final SparkMax deployMotor = new SparkMax(IntakeConstants.DEPLOY_ID, MotorType.kBrushless);
    private final WPI_TalonSRX rollerMotor = new WPI_TalonSRX(IntakeConstants.ROLLER_ID);

    private static DigitalInput limitSwitch = new DigitalInput(IntakeConstants.LIMIT_SWITCH_CHANNEL);

    private IntakeState state;

    public Intake() {
        deployMotor.configure(
                Configs.Intake.deployConfig,
                ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);
    }

    public void setIntakeState(IntakeState state) {
        this.state = state;
        setMotors();
        updateDashboard();
    }

    private void updateDashboard() {
        SmartDashboard.putString("IntakeState", state.toString());
        SmartDashboard.putBoolean("IntakeDeployed", isDeployed());
    }

    private void setMotors() {
        deployMotor.set(determineDeploySpeed());
        rollerMotor.set(determineRollerSpeed());
    }

    private double determineDeploySpeed() {
        if (state == IntakeState.DEPLOY && !isDeployed()) {
            return IntakeConstants.DEPLOY_SPEED;
        } else {
            return -IntakeConstants.DEPLOY_SPEED; // retracting at the same speed as deploying
        }
    }

    private double determineRollerSpeed() {
        if (state == IntakeState.DEPLOY) {
            return IntakeConstants.ROLLER_SPEED;
        } else {
            return 0.0;
        }
    }

    private boolean isDeployed() {
        return limitSwitch.get();
    }
}
