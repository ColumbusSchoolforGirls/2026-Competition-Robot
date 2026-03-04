package frc.robot.subsystems;

import frc.robot.Configs;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.hopper.Hopper;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;

public class ShootSystem {

    public enum ShooterState {
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
    private final SparkMax ventMotor = new SparkMax(ShooterConstants.VENT_ID, MotorType.kBrushless);

    private ShooterState state;
    private Hopper hopper;

    public ShootSystem(Hopper hopper) {
        this.hopper = hopper;
        state = ShooterState.STOPPED;

        ventMotor.configure(
                Configs.Shooter.ventConfig,
                ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);
    }

    public void setShooterState(ShooterState state) {
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
        ventMotor.set(determineVentPercentageOutput());
        setHopperState();
    }

    private double determineShooterRPM() {
        if (this.state == ShooterState.SHOOT || this.state == ShooterState.REV) {
            return ShooterConstants.SHOOT_RPM;
        }
        return 0.0;
    }

    private double determineFeederPercentageOutput(ShooterModule shooter) {
        if (this.state == ShooterState.SHOOT && isAtSpeed(shooter)) {
            return ShooterConstants.FEEDER_PERCENTAGE_OUTPUT;
        }
        return 0.0;
    }

    private double determineVentPercentageOutput() {
        if (this.state == ShooterState.SHOOT) {
            return ShooterConstants.VENT_PERCENTAGE_OUTPUT;
        }
        return 0.0;
    }

    private void setHopperState() {
        boolean run = (this.state == ShooterState.SHOOT);
        hopper.setHopper(run, false);
    }

    private boolean isAtSpeed(ShooterModule shooter) {
        return shooter.getShooterRPM() >= ShooterConstants.SHOOT_RPM - ShooterConstants.RPM_TOLERANCE
                && shooter.getShooterRPM() <= ShooterConstants.SHOOT_RPM + ShooterConstants.RPM_TOLERANCE;
    }
}