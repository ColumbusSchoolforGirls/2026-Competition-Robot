package frc.robot.auto;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;

import frc.robot.auto.states.AbstractAutoState;
import frc.robot.auto.states.AutoStateAlign;
import frc.robot.auto.states.AutoStateDrive;
import frc.robot.auto.states.AutoStateShoot;
import frc.robot.auto.states.AutoStateStop;
import frc.robot.auto.states.AutoTransition;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.limelight.Limelight;
import frc.robot.subsystems.shooter.ShootSystem;
import frc.robot.subsystems.hopper.Hopper;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

public class AutoStateMachine {
    private Drivetrain drivetrain;
    public AbstractAutoState currentAutoState;
    private Limelight limelight;
    private Hopper hopper;
    private ShootSystem shootSystem;

    public AutoStateMachine(Drivetrain drivetrain, Limelight limelight, ShootSystem shootSystem, Hopper hopper) {
        this.drivetrain = drivetrain;
        this.currentAutoState = null;
        this.limelight = limelight;
        this.shootSystem = shootSystem;
        this.hopper = hopper;
    }

    public enum StartingPosition {
        LEFT, MIDDLE, RIGHT // from drive station persepctive
    }

    // Shuffleboard
    ShuffleboardTab tab = Shuffleboard.getTab("Auto");
    GenericEntry leaveOnly, toHub, toClimb;

    private final SendableChooser<StartingPosition> positionChooser = new SendableChooser<>();

    private <K extends Enum<K>> void createChooser(SendableChooser<K> chooser, K[] values, String chooserName, int w,
            int h, int x, int y) {
        for (K value : values) {
            chooser.addOption(value.name(), value);
        }
        // SmartDashboard.putData(chooserName, chooser);
        tab.add(chooserName, chooser).withSize(w, h).withPosition(x, y);
    }

    // private boolean getIfSelected(GenericEntry entry) {
    // return entry.getBoolean(false);
    // }

    public void autoShuffleboardStartup() {
        createChooser(positionChooser, StartingPosition.values(), "Start Position", 2, 1, 0, 0);

        leaveOnly = tab.add("LEAVE ONLY", false).withWidget("Toggle Button").withSize(2, 1).withPosition(2, 0)
                .getEntry();

    }

    public AbstractAutoState buildPath() {
        // Set up all your States
        AutoStateStop start = new AutoStateStop(drivetrain);

        AutoStateDrive drive = new AutoStateDrive(1.17, 0, 0, drivetrain, -0.7);
        AutoStateShoot shoot = new AutoStateShoot(shootSystem, hopper, 5);
        AutoStateStop stop = new AutoStateStop(drivetrain);

        // Set up all your transitions
        start.addTransition(new AutoTransition(drive, state -> true));
        drive.addTransition(new AutoTransition(
                shoot, drive::atDistance));
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

}
