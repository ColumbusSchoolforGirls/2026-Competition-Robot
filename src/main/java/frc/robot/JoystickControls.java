package frc.robot;

import frc.robot.subsystems.ShootSystem;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Limelight;
import frc.robot.Constants.ControllerConstants;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.XboxController;

public class JoystickControls {
    private final Drivetrain drivetrain;
    private final Limelight limelight;
    private final ShootSystem shootSystem;
    private boolean fieldRelative = false;

    private static final XboxController DRIVE_CONTROLLER = new XboxController(0);
    private static final XboxController AUX = new XboxController(1);

    private final SlewRateLimiter xspeedLimiter = new SlewRateLimiter(3);
    private final SlewRateLimiter yspeedLimiter = new SlewRateLimiter(3);
    private final SlewRateLimiter rotLimiter = new SlewRateLimiter(3);

    public JoystickControls(Drivetrain drivetrain, Limelight limelight, ShootSystem shootSystem) {
        this.drivetrain = drivetrain;
        this.limelight = limelight;
        this.shootSystem = shootSystem;
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
        if (DRIVE_CONTROLLER.getAButton()) {
            final var rot_limelight = this.limelight.limelight_aim_proportional();
            rot = rot_limelight;

            final var forward_limelight = this.limelight.limelight_range_proportional();
            xSpeed = forward_limelight;

            // while using Limelight, turn off field-relative driving.
            fieldRelative = false;

            // TODO: this may not work... -Llama and Grace unsupervised
            // Gets to proper forward position, then stats spinning until apriltag out of
            // sight
            drivetrain.drive(xSpeed, 0, rot, fieldRelative, periodSeconds);
            System.out.println("CODE HAS BEEN REACHED, IT IS RUNNING");
        } else {
            drivetrain.drive(xSpeed, ySpeed, rot, fieldRelative, periodSeconds);
        }

        // // Get the x speed. We are inverting this because Xbox controllers return
        // // negative values when we push forward.
    }

    public void driverResetTurnEncoders() {
        if (DRIVE_CONTROLLER.getBButtonPressed()) {
            drivetrain.resetRelativeTurnEncoders();
        }
    }

    private ShootSystem.ShooterAction state = ShootSystem.ShooterAction.STOPPED;

    public void shoot() {
        switch (state) {
            case STOPPED: {
                if (AUX.getRightBumperButtonPressed()) {
                    state = ShootSystem.ShooterAction.REV;
                }
                break;
            }
            case REV: // Fall through intended
            case SHOOT: {
                if (AUX.getLeftBumperButtonPressed()) {
                    state = ShootSystem.ShooterAction.STOPPED;
                }

                if (AUX.getRightTriggerAxis() > ControllerConstants.TRIGGER_DEADZONE) {
                    state = ShootSystem.ShooterAction.SHOOT;
                } else {
                    state = ShootSystem.ShooterAction.REV;
                }

                break;
            }
            default: {
                state = ShootSystem.ShooterAction.STOPPED;
                break;
            }
        }

        shootSystem.setShooterState(state);
    }

}
