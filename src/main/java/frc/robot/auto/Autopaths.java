package frc.robot.auto;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;

import frc.robot.Constants.AutoConstants;
import frc.robot.auto.auto_states.AbstractAutoState;
import frc.robot.auto.auto_states.AutoStateDrive;
import frc.robot.auto.auto_states.AutoStateStop;
import frc.robot.auto.auto_states.AutoTransition;
import frc.robot.subsystems.Drivetrain;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
import java.util.function.Predicate;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

public class AutoPaths {
    private Drivetrain drivetrain;

    public AutoPaths(Drivetrain drivetrain) {
        this.drivetrain = drivetrain;
    }

    public enum StartingPosition {
        LEFT, MIDDLE, RIGHT // from drive station persepctive
    }

    public AbstractAutoState currentAutoState;

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

        this.currentAutoState = null;
    }

    public AbstractAutoState buildDrivePath() {
        HashMap<AbstractAutoState, AutoTransition> stateTransitionMap = new HashMap<>();

        AutoStateDrive driveForward1Meter = new AutoStateDrive(1, 0, 0, drivetrain, 1);
        AutoStateStop stop = new AutoStateStop();

        // Make our transition

        return driveForward1Meter;
    }

}
