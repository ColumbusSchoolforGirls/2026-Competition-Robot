package frc.robot.auto;

public class Autopaths {

    public static AutoStep[] autoLeftMain = {
            new AutoStep(AutoAction.ALIGNMENT, 0),
            new AutoStep(AutoAction.DRIVE, 0),
            new AutoStep(AutoAction.SHOOT, 0),
            new AutoStep(AutoAction.TURN, 0),
            new AutoStep(AutoAction.DRIVE, 0),
            new AutoStep(AutoAction.ALIGNMENT, 0),
            new AutoStep(AutoAction.CLIMB, 0)
    };
    public static AutoStep[] autoLeftShoot = {
            new AutoStep(AutoAction.ALIGNMENT, 0),
            new AutoStep(AutoAction.DRIVE, 0),
            new AutoStep(AutoAction.SHOOT, 0),
    };
    public static AutoStep[] autoLeftClimb = {
            new AutoStep(AutoAction.ALIGNMENT, 0),
            new AutoStep(AutoAction.DRIVE, 0),
            new AutoStep(AutoAction.ALIGNMENT, 0),
            new AutoStep(AutoAction.CLIMB, 0)
    };
    public static AutoStep[] autoMiddleMain = {
            new AutoStep(AutoAction.ALIGNMENT, 0),
            new AutoStep(AutoAction.SHOOT, 0),
            new AutoStep(AutoAction.TURN, 180),
            new AutoStep(AutoAction.ALIGNMENT, 0),
            new AutoStep(AutoAction.DRIVE, 0),
            new AutoStep(AutoAction.ALIGNMENT, 0),
            new AutoStep(AutoAction.CLIMB, 0),
    };
    public static AutoStep[] autoMiddleShoot = {
            new AutoStep(AutoAction.ALIGNMENT, 0),
            new AutoStep(AutoAction.SHOOT, 0)
    };
    public static AutoStep[] autoMiddleClimb = {
            new AutoStep(AutoAction.TURN, 180),
            new AutoStep(AutoAction.ALIGNMENT, 0),
            new AutoStep(AutoAction.DRIVE, 0),
            new AutoStep(AutoAction.ALIGNMENT, 0),
            new AutoStep(AutoAction.CLIMB, 0),
    };
    public static AutoStep[] autoRightMain = {
            new AutoStep(AutoAction.ALIGNMENT, 0),
            new AutoStep(AutoAction.DRIVE, 0),
            new AutoStep(AutoAction.SHOOT, 0),
            new AutoStep(AutoAction.TURN, 0),
            new AutoStep(AutoAction.DRIVE, 0),
            new AutoStep(AutoAction.ALIGNMENT, 0),
            new AutoStep(AutoAction.CLIMB, 0),
    };
    public static AutoStep[] autoRightShoot = {
            new AutoStep(AutoAction.ALIGNMENT, 0),
            new AutoStep(AutoAction.DRIVE, 0),
            new AutoStep(AutoAction.SHOOT, 0),
    };
    public static AutoStep[] autoRightClimb = {
            new AutoStep(AutoAction.ALIGNMENT, 0),
            new AutoStep(AutoAction.DRIVE, 0),
            new AutoStep(AutoAction.ALIGNMENT, 0),
            new AutoStep(AutoAction.CLIMB, 0)
    };
}
