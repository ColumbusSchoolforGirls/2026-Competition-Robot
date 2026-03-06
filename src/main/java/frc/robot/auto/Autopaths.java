package frc.robot.auto;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;

import frc.robot.Constants.AutoConstants;
import frc.robot.auto.states.AbstractAutoState;
import frc.robot.auto.states.AutoStateAlign;
import frc.robot.auto.states.AutoStateDrive;
import frc.robot.auto.states.AutoStateStop;
import frc.robot.auto.states.AutoTransition;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Limelight;

import java.util.HashMap;
import java.util.function.Predicate;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

public class AutoPaths {
    private Drivetrain drivetrain;
    public AbstractAutoState currentAutoState;
    private Limelight limelight;

    public AutoPaths(Drivetrain drivetrain, Limelight limelight) {
        this.drivetrain = drivetrain;
        this.currentAutoState = null;
        this.limelight = limelight;
    }

    public enum StartingPosition {
        LEFT, MIDDLE, RIGHT // from drive station persepctive
    }

    // Shuffleboard
    ShuffleboardTab tab = Shuffleboard.getTab("Auto");
    GenericEntry leaveOnly, toHub, toDepot, toClimb, toCenterField, shoot;

    private final SendableChooser<StartingPosition> positionChooser = new SendableChooser<>();

    private <K extends Enum<K>> void createChooser(SendableChooser<K> chooser, K[] values, String chooserName, int w,
            int h, int x, int y) {
        for (K value : values) {
            chooser.addOption(value.name(), value);
        }
        // SmartDashboard.putData(chooserName, chooser);
        tab.add(chooserName, chooser).withSize(w, h).withPosition(x, y);
    }

    private boolean getIfSelected(GenericEntry entry) {
        return entry.getBoolean(false);
    }

    public void autoShuffleboardStartup() {
        createChooser(positionChooser, StartingPosition.values(), "Start Position", 2, 1, 0, 0);

        leaveOnly = tab.add("LEAVE ONLY", false).withWidget("Toggle Button").withSize(2, 1).withPosition(2, 0)
                .getEntry();

    }

    public AbstractAutoState buildPath() {
        // Set up all your States
        AutoStateStop start = new AutoStateStop();

        // TODO: THESE ARE PLACEHOLDERS
        AutoStateDrive driveTurnRight = new AutoStateDrive(0, 0, 0, drivetrain, 0);
        AutoStateDrive driveTurnLeft = new AutoStateDrive(0, 0, 0, drivetrain, 0);

        AutoStateDrive driveForward1Meter = new AutoStateDrive(1, 0, 0, drivetrain, 1);
        AutoStateAlign align = new AutoStateAlign(null, drivetrain);

        AutoStateStop stop = new AutoStateStop();

        // Set up all your transitions
        start.addTransition(new AutoTransition(
                driveTurnLeft, state -> positionChooser.getSelected() == StartingPosition.RIGHT));
        start.addTransition(new AutoTransition(
                driveTurnRight, state -> positionChooser.getSelected() == StartingPosition.LEFT));

        driveTurnRight.addTransition(new AutoTransition(
                align, driveTurnRight::atDistance));
        driveTurnLeft.addTransition(new AutoTransition(
                align, driveTurnLeft::atDistance));

        align.addTransition(new AutoTransition(stop, align::isAligned));

        return driveForward1Meter;
    }

    // Check for transitions first, otherwise run the action function.
    public void runStateMachine(AbstractAutoState startState, double periodSeconds) {
        AbstractAutoState nextState = currentAutoState.getNextState();
        if (nextState != null) {
            currentAutoState = nextState;
            nextState.startState();
            return;
        }
        currentAutoState.action(periodSeconds);
    }

}
