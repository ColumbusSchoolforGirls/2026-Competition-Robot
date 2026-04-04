package frc.robot.subsystems.shooter;

import frc.robot.Configs;
import frc.robot.Constants.ShooterConstants;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;

public class ShootSystem {

    public enum ShooterState {
        STOPPED, REV, SHOOT
    }

    private final SlewRateLimiter shootSpeedLimiter = new SlewRateLimiter(750);

    private boolean runVentAgainstIntake;
    private boolean runVentToExpelIntake;

    private final ShooterModule leftShooter = new ShooterModule(
            ShooterConstants.LEFT_LEAD_ID,
            ShooterConstants.LEFT_FOLLOWER_ID,
            ShooterConstants.LEFT_FEEDER_ID);
    private final ShooterModule rightShooter = new ShooterModule(
            ShooterConstants.RIGHT_LEAD_ID,
            ShooterConstants.RIGHT_FOLLOWER_ID,
            ShooterConstants.RIGHT_FEEDER_ID);
    private final SparkFlex ventMotor = new SparkFlex(ShooterConstants.VENT_ID, MotorType.kBrushless);

    private ShooterState state;

    public ShootSystem() {
        state = ShooterState.STOPPED;
        runVentAgainstIntake = false;
        runVentToExpelIntake = false;

        ventMotor.configure(
                Configs.Shooter.ventConfig,
                ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);
    }

    public void init() {
        state = ShooterState.STOPPED;
    }

    public void setShooterState(ShooterState state) {
        this.state = state;
        setMotors();
        updateDashboard();
    }

    public void runVentAgainstIntake(boolean runVentAgainstIntake) {
        this.runVentAgainstIntake = runVentAgainstIntake;
    }

    public void runVentToExpelIntake(boolean runVentToExpelIntake) {
        this.runVentToExpelIntake = runVentToExpelIntake;
    }

    public void updateDashboard() {
        SmartDashboard.putString("ShooterState", state.toString());
        SmartDashboard.putNumber("Left Shooter RPM", leftShooter.getShooterRPM());
        SmartDashboard.putNumber("Right Shooter RPM", rightShooter.getShooterRPM());
        SmartDashboard.putBoolean("Left Shooter At Speed", isAtSpeed(leftShooter));
        SmartDashboard.putBoolean("Right Shooter At Speed", isAtSpeed(rightShooter));

        SmartDashboard.putNumber("Right Lead Shoot Velocity", rightShooter.getLeadMotor().getEncoder().getVelocity());
        SmartDashboard.putNumber("Right Lead Shoot Voltage",
                rightShooter.getLeadMotor().getAppliedOutput() * rightShooter.getLeadMotor().getBusVoltage());

    }

    private void setMotors() {
        leftShooter.setShooterRPM(shootSpeedLimiter.calculate(determineShooterRPM()));
        rightShooter.setShooterRPM(shootSpeedLimiter.calculate(determineShooterRPM() - 100));
        leftShooter.setFeeder(determineFeederPercentageOutput(leftShooter));
        rightShooter.setFeeder(determineFeederPercentageOutput(rightShooter));
        ventMotor.set(determineVentPercentageOutput());
    }

    public void autoSetVent() {
        ventMotor.set(ShooterConstants.VENT_PERCENTAGE_OUTPUT);
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
        } else if (runVentAgainstIntake) {
            return ShooterConstants.VENT_AGAINST_INTAKE_PRECENTAGE_OUTPUT;
        } else if (runVentToExpelIntake) {
            return ShooterConstants.VENT_EXPEL_INTAKE_PERCENTAGE_OUTPUT;
        }
        return 0.0;
    }

    private boolean isAtSpeed(ShooterModule shooter) {
        return shooter.getShooterRPM() >= ShooterConstants.SHOOT_RPM - ShooterConstants.RPM_TOLERANCE
                && shooter.getShooterRPM() <= ShooterConstants.SHOOT_RPM + ShooterConstants.RPM_TOLERANCE;
    }
}