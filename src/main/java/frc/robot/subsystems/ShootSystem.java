package frc.robot.subsystems;

import frc.robot.Configs;
import frc.robot.Constants.ShooterConstants;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;

public class ShootSystem {

    public enum ShooterAction {
        STOPPED, REV, SHOOT
    }

    private final ShooterModule leftShooter = new ShooterModule(
            ShooterConstants.LEFT_LEAD_ID,
            ShooterConstants.LEFT_FOLLOWER_ID,
            ShooterConstants.LEFT_FEEDER_ID);
    private final ShooterModule rightShooter = new ShooterModule(
            ShooterConstants.RIGHT_LEAD_ID,
            ShooterConstants.RIGHT_FOLLOWER_ID,
            ShooterConstants.RIGHT_FEEDER_ID);
    private final SparkMax rollersMotor = new SparkMax(ShooterConstants.ROLLERS_ID, MotorType.kBrushless);

    private ShooterAction state;

    public ShootSystem() {
        state = ShooterAction.STOPPED;

        rollersMotor.configure(
                Configs.Shooter.rollersConfig,
                ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);
    }

    public void setShooterState(ShooterAction state) {
        this.state = state;
        setMotors();
        updateDashboard();
    }

    private void updateDashboard() {
        SmartDashboard.putString("ShooterState", state.toString());
        SmartDashboard.putNumber("Left Shooter RPM", leftShooter.getShooterRPM());
        SmartDashboard.putNumber("Right Shooter RPM", rightShooter.getShooterRPM());
        SmartDashboard.putBoolean("Left Shooter At Speed", isAtSpeed(leftShooter));
        SmartDashboard.putBoolean("Right Shooter At Speed", isAtSpeed(rightShooter));
    }

    private void setMotors() {
        leftShooter.setShooterRPM(determineShooterRPM());
        rightShooter.setShooterRPM(determineShooterRPM());
        leftShooter.setFeeder(determineFeederPercentageOutput(leftShooter));
        rightShooter.setFeeder(determineFeederPercentageOutput(rightShooter));
        rollersMotor.set(determineRollersPercentageOutput());
    }

    private double determineShooterRPM() {
        if (this.state == ShooterAction.SHOOT || this.state == ShooterAction.REV) {
            return ShooterConstants.SHOOT_RPM;
        }
        return 0.0;
    }

    private double determineFeederPercentageOutput(ShooterModule shooter) {
        if (this.state == ShooterAction.SHOOT && isAtSpeed(shooter)) {
            return ShooterConstants.FEEDER_PERCENTAGE_OUTPUT;
        }
        return 0.0;
    }

    private double determineRollersPercentageOutput() {
        if (this.state == ShooterAction.SHOOT) {
            return ShooterConstants.ROLLERS_PERCENTAGE_OUTPUT;
        }
        return 0.0;
    }

    private boolean isAtSpeed(ShooterModule shooter) {
        return shooter.getShooterRPM() >= ShooterConstants.SHOOT_RPM - ShooterConstants.RPM_TOLERANCE
                && shooter.getShooterRPM() <= ShooterConstants.SHOOT_RPM + ShooterConstants.RPM_TOLERANCE;
    }
}