package frc.robot;

import frc.robot.Constants.ControllerConstants;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.hopper.Hopper;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.ShootSystem;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.XboxController;

/*
 * List all controls in this docstring.
 * DRIVER:
 *     * Driving
 *         - Left Joystick: Linear driving movement (omnidirectional)
 *             - (Press): Toggle field-relative driving
 *         - Right Joystick: Rotational movement
 *             - (Press): Reset gyro
 *         - Left Bumper (Hold): Crawl mode (reduce driving speed)
 *         - A Button (Press): Align with limelight
 *         - B Button (Press): Reset turn encoders and gyro
 *     * Climber
 *         - Y Button (Hold): Climber goes up; extend
 *         - X Button (Hold): Climber climbs/goes down; flex
 * 
 * AUXILIARY:
 *     * Shooter
 *         - Left Bumper (Press): Stop Shooter
 *         - Right Bumper (Press): Rev Shooter
 *         - Right Trigger (Hold): Shoot
 *     * Intake
 *         - Left Trigger (Hold): Intake
 *         - Y Button (Hold): Empty Intake
 *         - X Button (Press): Deploy Intake
 *         - B Button (Press): Retract Intake
 *     NOTE: The hopper automatically runs during intaking and shooting.
 * 
 * RESET CONTROLLER:
 *     - Right Bumper (Press): Toggle climber brake/coast mode
 *     - B Button (Hold): Drive climber down
 *     - Y Button (Hold): Drive climber up
 * 
 * SYSTEM TEST CONTROLLER:
 * 
 */
public class JoystickControls {
    private final XboxController DRIVE_CONTROLLER = new XboxController(0); // TODO: Is static needed here?
    private final XboxController AUX = new XboxController(1);
    private final XboxController RESET_CONTROLLER = new XboxController(2);
    private final XboxController SYSTEM_TEST_CONTROLLER = new XboxController(3);

    private final Drivetrain drivetrain;
    private final ShootSystem shootSystem;
    private final Intake intake;
    private final Hopper hopper;
    private final Climber climber;

    // TODO: Tune the limits (probably lower to prevent burnout)
    private final SlewRateLimiter xspeedLimiter = new SlewRateLimiter(3);
    private final SlewRateLimiter yspeedLimiter = new SlewRateLimiter(3);
    private final SlewRateLimiter rotLimiter = new SlewRateLimiter(3);

    private ShootSystem.ShooterState shootState = ShootSystem.ShooterState.STOPPED;
    private boolean fieldRelative = true;
    private boolean runHopperForShooting = false;
    private boolean runHopperForIntaking = false;
    private boolean expelSystem = false;

    public JoystickControls(Drivetrain drivetrain, ShootSystem shootSystem, Intake intake,
            Hopper hopper, Climber climber) {
        this.drivetrain = drivetrain;
        this.shootSystem = shootSystem;
        this.intake = intake;
        this.hopper = hopper;
        this.climber = climber;
    }

    public void init() {
        shootState = ShootSystem.ShooterState.STOPPED;
    }

    public void testTurn(double periodSeconds) {
        drivetrain.runTurn();
    }

    public void driveWithJoystick(double periodSeconds) {
        double xSpeed = -xspeedLimiter.calculate(MathUtil.applyDeadband(DRIVE_CONTROLLER.getLeftY(), 0.1))
                * Constants.DriveConstants.MAX_SPEED;
        double ySpeed = yspeedLimiter.calculate(MathUtil.applyDeadband(DRIVE_CONTROLLER.getLeftX(), 0.1))
                * Constants.DriveConstants.MAX_SPEED;
        double rot = rotLimiter
                .calculate(MathUtil.applyDeadband(DRIVE_CONTROLLER.getRightX(), ControllerConstants.JOYSTICK_DEADZONE))
                * Constants.DriveConstants.MAX_ANGULAR_SPEED;

        if (DRIVE_CONTROLLER.getLeftBumperButton()) {
            xSpeed *= Constants.DriveConstants.CRAWL_SPEED;
            ySpeed *= Constants.DriveConstants.CRAWL_SPEED;
            rot *= Constants.DriveConstants.CRAWL_SPEED;
        }

        if (DRIVE_CONTROLLER.getLeftStickButtonPressed()) {
            fieldRelative = !fieldRelative;
        }

        if (DRIVE_CONTROLLER.getRightStickButtonPressed()) {
            drivetrain.resetGyro();
        }

        if (DRIVE_CONTROLLER.getBButtonPressed()) {
            drivetrain.resetRelativeTurnEncoders();
        }

        // Overwrite drive with limelight alignment.
        if (DRIVE_CONTROLLER.getAButton()) {
            drivetrain.autoAlign(periodSeconds);
            return;
        }

        drivetrain.drive(xSpeed, ySpeed, rot, fieldRelative, periodSeconds);
    }

    public void shoot() {
        expelSystem = AUX.getYButton();

        switch (shootState) {
            case STOPPED: {
                if (AUX.getRightBumperButtonPressed()) {
                    shootState = ShootSystem.ShooterState.REV;
                }
                break;
            }
            case REV: // Fall through intended
            case SHOOT: {
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

        if (AUX.getLeftBumperButtonPressed()) {
            shootState = ShootSystem.ShooterState.STOPPED;
        }

        runHopperForShooting = shootSystem.runHopper();
        shootSystem.expelSystem(expelSystem);
        shootSystem.setShooterState(shootState);
    }

    public void intake() {
        boolean runRollers = AUX.getLeftTriggerAxis() > ControllerConstants.JOYSTICK_DEADZONE;
        runHopperForIntaking = runRollers;
        expelSystem = AUX.getYButton();

        if (AUX.getXButton()) {
            intake.deploy();
        } else if (AUX.getBButton()) {
            intake.retract();
            runRollers = true;
        } else {
            intake.stopDeploy();
        }

        intake.runRollers(runRollers);
        intake.expelSystem(expelSystem);
        intake.setIntakeRollers();
    }

    public void hopper() {
        hopper.expelHopper(expelSystem);
        hopper.runHopper(runHopperForShooting || runHopperForIntaking);
        hopper.setHopper();
    }

    public void climber() {
        if (DRIVE_CONTROLLER.getYButton()) { // raise the climber
            climber.extendClimber();
        } else if (DRIVE_CONTROLLER.getXButton()) { // flex the climber (climb)
            climber.flexClimber();
        } else if (DRIVE_CONTROLLER.getBButton()) { // lower the climber
            climber.retractClimber();
        } else if (RESET_CONTROLLER.getYButton()) {
            climber.driveUpForReset();
        } else if (RESET_CONTROLLER.getBButton()) {
            climber.driveDownForReset();
        } else {
            climber.stopClimber();
        }

        if (RESET_CONTROLLER.getRightBumperButtonPressed()) {
            climber.lockClimbMotor();
        } else {
            climber.unlockClimbMotor();
        }
    }
}