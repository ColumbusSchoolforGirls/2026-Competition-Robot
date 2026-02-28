package frc.robot.auto;

import edu.wpi.first.wpilibj.TimedRobot;

import java.util.ArrayList;

import frc.robot.Robot;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.ShootSystem;
// import frc.robot.subsystems.Intake;

public class AutoState {
    private Robot robot;
    private AutoStep currentAction = new AutoStep(AutoAction.STOP);
    private Drivetrain drivetrain;
    private Limelight limelight;
    private ShootSystem shootSystem;
    // private Intake intake;

    private int state;

    private ArrayList<AutoStep> autoActions = new ArrayList<>();

    public AutoState(Robot robot, Drivetrain drivetrain, Limelight limelight, ShootSystem shootSystem) { // Intake
        this.robot = robot;
        this.drivetrain = drivetrain;
        this.limelight = limelight;
        this.shootSystem = shootSystem;
        // this.intake = intake;

        state = -1;
    }

    public void goToNextState() {
        state++;
        if (state >= autoActions.size()) {
            System.out.println("End of auto path :)");
        }
    }

    public void getForNextState() {
        switch (currentAction.getAction()) {
            case DRIVE:
                drivetrain.startDrive(currentAction.getValue());
                break;
            case TURN:
                drivetrain.startTurn(currentAction.getValue());
                break;
            case ALIGN:
                break;
            // case SHOOT:
            // shootSystem.autoShootStart();
            // break;
            // case INTAKE:
            // intake.autoIntakeStart();
            // break;
            // case CLIMB:
            // climb.autoClimbStart();
            // break;
            case STOP:
                break;
            default:
                break;
        }
    }

    public void getAutoState() {
        switch (currentAction.getAction()) {
            case DRIVE:
                if (drivetrain.driveComplete()) {
                    goToNextState();
                } else {
                    drivetrain.autoDrive(robot.getPeriod());
                }
                break;
            case TURN:
                if (drivetrain.turnComplete()) {
                    goToNextState();
                } else {
                    drivetrain.gyroTurn(robot.getPeriod());
                    System.out.println(drivetrain.getHeading());
                }
                break;
            // case SHOOT:
            // if (shootSystem.autoShootComplete()) {
            // goToNextState();
            // } else {
            // shootSystem.autoShoot();
            // }
            // break;
            case ALIGN:
                if (drivetrain.alignComplete()) {
                    goToNextState();
                } else {
                    drivetrain.autoAlign(robot.getPeriod());
                }
                break;
            // case INTAKE:
            // if (intake.autoIntakeComplete()) {
            // goToNextState();
            // } else {
            // intake.autoIntake();
            // }
            // break;
            // case CLIMB:
            // if (climber.autoClimbComplete()) {
            // goToNextState();
            // } else {
            // climber.autoClimb();
            // }
            // break;
            case STOP:
                drivetrain.stop(robot.getPeriod());
                break;
            default:
                break;
        }
    }
}
