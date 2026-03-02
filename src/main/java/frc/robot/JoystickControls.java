package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Constants.ControllerConstants;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.ShootSystem;
import frc.robot.subsystems.Intake.Intake;
import frc.robot.subsystems.Intake.IntakeState;

/*
 * List all controls in this docstring.
 * DRIVER:
 *     - Left Joystick: Linear driving movement (omnidirectional)
 *     - Right Joystick: Rotational movement
 *     - Left Bumper (Hold): Crawl mode (reduce driving speed)
 *     - A Button (Press): Align with limelight
 *     - B Button (Press): Reset turn encoders
 * 
 * AUXILIARY:
 *     * Shooter
 *         - Left Bumper (Press): Stop Shooter
 *         - Right Bumper (Press): Rev Shooter
 *         - Right Trigger (Hold): Shoot
 *     * Intake
 *         - Left Trigger (Hold): Intake
 *         - X Button (Press): Deploy Intake
 *         - B Button (Press): Retract Intake
 */
public class JoystickControls {
    private static final XboxController DRIVE_CONTROLLER = new XboxController(0);
    private static final XboxController AUX = new XboxController(1);

    private final Drivetrain drivetrain;
    private final Limelight limelight;
    private final ShootSystem shootSystem;
    private final Intake intake;

    private final SlewRateLimiter xspeedLimiter = new SlewRateLimiter(3);
    private final SlewRateLimiter yspeedLimiter = new SlewRateLimiter(3);
    private final SlewRateLimiter rotLimiter = new SlewRateLimiter(3);

    private boolean fieldRelative = false;
    private ShootSystem.ShooterState shootState = ShootSystem.ShooterState.STOPPED;
    private IntakeState intakeState = IntakeState.IN;

    public JoystickControls(Drivetrain drivetrain, Limelight limelight, ShootSystem shootSystem, Intake intake) {
        this.drivetrain = drivetrain;
        this.limelight = limelight;
        this.shootSystem = shootSystem;
        this.intake = intake;
    }

    public void driveWithJoystick(double periodSeconds) {
        double xSpeed = -xspeedLimiter.calculate(MathUtil.applyDeadband(DRIVE_CONTROLLER.getLeftY(), 0.1))
                * Constants.DriveConstants.MAX_SPEED;

        // Get the y speed or sideways/strafe speed. We are inverting this because
        // we want a positive value when we pull to the left. Most controllers
        // return positive values when you pull to the right by default. // nah positive
        // now
        double ySpeed = yspeedLimiter.calculate(MathUtil.applyDeadband(DRIVE_CONTROLLER.getLeftX(), 0.1))
                * Constants.DriveConstants.MAX_SPEED;

        // Get the rate of angular rotation. We are inverting this because we want a
        // positive value when we pull to the left (remember, CCW is positive in
        // mathematics). Xbox controllers return positive values when you pull to
        // the right by default. // uh no, positive now
        double rot = rotLimiter
                .calculate(MathUtil.applyDeadband(DRIVE_CONTROLLER.getRightX(), ControllerConstants.JOYSTICK_DEADZONE))
                * Constants.DriveConstants.MAX_ANGULAR_SPEED;

        if (DRIVE_CONTROLLER.getLeftBumperButton()) {
            xSpeed *= Constants.DriveConstants.CRAWL_SPEED; // if you hold the left bumper you go at a slow scaled
                                                            // speed
            ySpeed *= Constants.DriveConstants.CRAWL_SPEED;
            rot *= Constants.DriveConstants.CRAWL_SPEED;
        }

        // while the A-button is pressed, overwrite some of the driving values with the
        // output of our limelight methods
        if (DRIVE_CONTROLLER.getXButton()) {
            // while using Limelight, turn off field-relative driving.
            fieldRelative = false;

            final var rot_limelight = this.limelight.limelight_aim_proportional();
            rot = rot_limelight;

            final var forward_limelight = this.limelight.limelight_range_proportional();
            xSpeed = forward_limelight;

            drivetrain.drive(xSpeed, ySpeed, rot_limelight, fieldRelative, periodSeconds);
        } else {
            fieldRelative = true;

            drivetrain.drive(xSpeed, ySpeed, rot, fieldRelative, periodSeconds);
        }

        // // Get the x speed. We are inverting this because Xbox controllers return
        // // negative values when we push forward.
    }

    public void driverResetTurnEncoders() {
        if (DRIVE_CONTROLLER.getAButtonPressed()) {
            drivetrain.resetRelativeTurnEncoders();
        }
    }

    public void driverResetGyro() {
        if (DRIVE_CONTROLLER.getBButton())
            drivetrain.resetGyro();
    }

    private ShootSystem.ShooterState state = ShootSystem.ShooterState.STOPPED;

    public void shoot() {
        switch (shootState) {
            case STOPPED: {
                if (AUX.getRightBumperButtonPressed()) {
                    shootState = ShootSystem.ShooterState.REV;
                }
                break;
            }
            case REV: // Fall through intended
            case SHOOT: {
                if (AUX.getLeftBumperButtonPressed()) {
                    shootState = ShootSystem.ShooterState.STOPPED;
                }

                if (AUX.getRightTriggerAxis() > ControllerConstants.TRIGGER_DEADZONE) {
                    shootState = ShootSystem.ShooterState.SHOOT;
                } else {
                    shootState = ShootSystem.ShooterState.REV;
                }

                break;
            }
            default: {
                shootState = ShootSystem.ShooterState.STOPPED;
                break;
            }
        }

        shootSystem.setShooterState(shootState);
    }

    public void intake() {
        boolean runRoller = AUX.getLeftTriggerAxis() > ControllerConstants.JOYSTICK_DEADZONE;

        switch (intakeState) {
            case IN: {
                if (AUX.getXButtonPressed()) {
                    intakeState = IntakeState.DEPLOYING;
                }
                break;
            }
            case OUT: {
                if (AUX.getBButtonPressed()) {
                    intakeState = IntakeState.RETRACTING;
                    break;
                }
                break;
            }
            case DEPLOYING: {
                if (intake.isDeployed()) {
                    intakeState = IntakeState.OUT;
                }
                break;
            }
            case RETRACTING: {
                if (intake.isRetracted()) {
                    intakeState = IntakeState.IN;
                }
                break;
            }
            default: {
            }
        }
        intake.setIntakeState(intakeState, runRoller);
    }

}
