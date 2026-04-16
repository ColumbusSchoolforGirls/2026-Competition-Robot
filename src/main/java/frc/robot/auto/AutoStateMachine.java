package frc.robot.auto;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import frc.robot.auto.states.AbstractAutoState;
import frc.robot.auto.states.AutoStateDrive;
import frc.robot.auto.states.AutoStateRev;
import frc.robot.auto.states.AutoStateIntake;
import frc.robot.auto.states.AutoStateShoot;
import frc.robot.auto.states.AutoStateStop;
import frc.robot.auto.states.AutoTransition;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.limelight.Limelight;
import frc.robot.subsystems.shooter.ShootSystem;
import frc.robot.subsystems.hopper.Hopper;
import frc.robot.subsystems.intake.Intake;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class AutoStateMachine {
    private Drivetrain drivetrain;
    public AbstractAutoState currentAutoState;
    private Limelight limelight;
    private Hopper hopper;
    private ShootSystem shootSystem;
    private Intake intake;

    public AutoStateMachine(Drivetrain drivetrain, Limelight limelight, ShootSystem shootSystem, Hopper hopper,
            Intake intake) {
        this.drivetrain = drivetrain;
        this.currentAutoState = null;
        this.limelight = limelight;
        this.shootSystem = shootSystem;
        this.hopper = hopper;
        this.intake = intake;
    }

    public enum StartingPosition {
        LEFT, MIDDLE, RIGHT // from drive station persepctive
    }

    // SmartDashboard Choosers (theoretically)
    // SmartDashboard.putData("Button Name", new Command());
    // //TODO: I WANT THIS FOR SHOOTING BUT WILL RUIN A LOT OF THINGS

    private final SendableChooser<StartingPosition> positionChooser = new SendableChooser<>();

    public void autoDashboardStartup() {

        positionChooser.setDefaultOption("Middle", StartingPosition.MIDDLE);
        positionChooser.addOption("Left", StartingPosition.LEFT);
        positionChooser.addOption("Right", StartingPosition.RIGHT);

        SmartDashboard.putData("Starting Position", positionChooser);
    }

    public AbstractAutoState buildPath() {
        // Set up all your States
        AutoStateStop start = new AutoStateStop(drivetrain);

        AutoStateRev rev = new AutoStateRev(shootSystem);
        AutoStateIntake intaking = new AutoStateIntake(intake, true);
        AutoStateIntake stopIntake = new AutoStateIntake(intake, false);
        AutoStateDrive driveBack = new AutoStateDrive(1.17, 0, 0, drivetrain, -2);
        AutoStateShoot shoot = new AutoStateShoot(shootSystem, hopper, 3);

        AutoStateDrive spin = new AutoStateDrive(0, 0, 215, drivetrain, 2.6);
        AutoStateDrive drive = new AutoStateDrive(4, 0, 0, drivetrain, 2.5);

        AutoStateDrive driveReturn = new AutoStateDrive(3.6, 0, 0, drivetrain, -2.6);
        AutoStateDrive turnReturn = new AutoStateDrive(0, 0, 165, drivetrain, 2.5);

        AutoStateStop stop = new AutoStateStop(drivetrain);

        // Set up all your transitions
        start.addTransition(new AutoTransition(rev, state -> true));
        rev.addTransition(new AutoTransition(driveBack, state -> true));

        driveBack.addTransition(new AutoTransition(
                shoot, driveBack::atDistance));
        shoot.addTransition(new AutoTransition(spin180, shoot::atTime));

        spin180.addTransition(new AutoTransition(intaking, state -> spin180.atAngle(state)));
        intaking.addTransition(new AutoTransition(drive, state -> true));
        drive.addTransition(new AutoTransition(stopIntake, drive::atDistance));
        stopIntake.addTransition(new AutoTransition(driveReturn, state -> true));
        driveReturn.addTransition(new AutoTransition(turnReturn, driveReturn::atDistance));
        turnReturn.addTransition(new AutoTransition(shoot, turnReturn::atAngle));
        shoot.addTransition(new AutoTransition(stop, shoot::atTime));
        return start;
    }

    // Check for transitions first, otherwise run the action function.
    public void runStateMachine(AbstractAutoState startState, double periodSeconds) {
        if (currentAutoState == null) {
            currentAutoState = startState;
        }
        AbstractAutoState nextState = currentAutoState.getNextState();
        if (nextState != null) {
            currentAutoState.onStateExit(periodSeconds);
            currentAutoState = nextState;
            nextState.onStateEntry(periodSeconds);
            return;
        }
        currentAutoState.action(periodSeconds);
    }

    public void updateDashboard() {
        SmartDashboard.putString("Current Auto State", currentAutoState.toString());
    }
}
