package frc.robot.auto;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import frc.robot.Constants.AutoConstants;
import edu.wpi.first.networktables.GenericEntry;

import java.util.ArrayList;
import java.util.Arrays;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;

public class AutoPaths {

    public enum StartingPosition {
        LEFT, MIDDLE, RIGHT // from drive station persepctive
    }

    public AutoAction currentAutoAction;

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

        this.currentAutoAction = null;
    }

    public ArrayList<AutoStep> buildPath() {
        ArrayList<AutoStep> path = new ArrayList<>();

        if (getIfSelected(leaveOnly)) {
            path.add(new AutoStep(AutoAction.DRIVE, AutoConstants.LEAVE_ONLY_DISTANCE));
            return path;
        }
        if (getIfSelected(toHub)) {
            return path;
        }
        if (getIfSelected(toClimb)) {
            return path;
        }
        if (getIfSelected(toDepot)) {
            return path;
        }
        if (getIfSelected(toCenterField)) {
            return path;
        }
        if (getIfSelected(shoot)) {
            return path;
        }
        return path;
    }
}
