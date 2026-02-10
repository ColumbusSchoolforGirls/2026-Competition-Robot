package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import static frc.robot.Constants.ControllerConstants.AUX;
import frc.robot.Constants.ShooterConstants;

public class Shooter {

    enum ShooterAction {
        STOP, REV, SHOOT
    }
    ShooterAction state = ShooterAction.STOP;
    String shooterState = "STOP";

    private final SparkMax shooterMotor = new SparkMax(ShooterConstants.SHOOTER_ID, MotorType.kBrushless);
    private final SparkMax feederMotor = new SparkMax(ShooterConstants.FEEDER_ID, MotorType.kBrushless);

    private RelativeEncoder shooterEncoder = shooterMotor.getEncoder();
    private RelativeEncoder feederEncoder = feederMotor.getEncoder();

    private SparkClosedLoopController shooterPidController;
    private final SparkBaseConfig config = new SparkMaxConfig();

    private double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput, maxRPM;


    public Shooter() {
        this.shooterPidController = shooterMotor.getClosedLoopController();
        setCurrentLimit();
        setCoastMode();
    }

    public void update() {
        shoot();
    }

    public void shoot() {
        shooterState = this.state.toString();

        if (AUX.getYButtonPressed()) {
            this.state = ShooterAction.STOP;
        }

        switch(this.state) {
            case STOP: {
                setMotors();
                if (AUX.getRightBumperButtonPressed()) {
                    this.state = ShooterAction.REV;
                }
            }
            case REV: {
                setMotors();
                if (AUX.getRightTriggerAxis() > 0.5) {
                    this.state = ShooterAction.SHOOT;
                }
            }
            case SHOOT: {
                setMotors();
            }
            default: {
                setMotors();
            }
        }
    }

    private void setMotors() {
        feederMotor.set(getFeederSpeed());
        shooterPidController.setReference(getShooterSpeed(), SparkMax.ControlType.kVelocity);
    }

    private double getShooterSpeed() {
        if(this.state == ShooterAction.SHOOT || this.state == ShooterAction.REV) {
            return ShooterConstants.SHOOT_SPEED;
        }
        return 0.0;
    }

    private double getFeederSpeed() {
        if (this.state == ShooterAction.SHOOT) {
            return ShooterConstants.SHOOT_SPEED;
        }
        return 0.0;
    }

    private void stopMotors() {
        shooterMotor.set(0);
        feederMotor.set(0);
    }

    // TODO: Tune the PID
    public void setUpMotors() {
        shooterPidController = shooterMotor.getClosedLoopController();

        SparkMaxConfig setupConfig = new SparkMaxConfig();
        kP = 0.00015;
        kI = 0;
        kD = 0;
        kIz = 0;
        kFF = 0.000172;
        kMaxOutput = 1;
        kMinOutput = -1;
        maxRPM = 5700;
        
        setupConfig.idleMode(IdleMode.kCoast);
        // setupConfig.encoder
        //     .velocityConversionFactor(1000); // TODO: see if needed
        setupConfig.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .pidf(kP, kI, kD, kFF)
            .outputRange(kMinOutput, kMaxOutput);
                
        shooterMotor.configure(setupConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
        feederMotor.configure(setupConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    }

    // TODO: Update limits with testing, and add the feedermotor if needed
    public void setCurrentLimit() { // Limits may need to be changed
        config.smartCurrentLimit(30, 40, 1);
        shooterMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        feederMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    // Always run shooters in coast mode
    private void setCoastMode() {
        config.idleMode(SparkBaseConfig.IdleMode.kCoast);
        shooterMotor.configure(config, SparkBase.ResetMode.kNoResetSafeParameters, SparkBase.PersistMode.kNoPersistParameters);
    }
}