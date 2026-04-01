package frc.robot.auto.states;

import frc.robot.Constants.ClimberConstants;
import frc.robot.subsystems.climber.Climber;

public class AutoStateClimb extends AbstractAutoState {

    private enum ClimbingState {
        REST(ClimberConstants.REST_HEIGHT_ROTATIONS),
        SETUP(ClimberConstants.MAX_HEIGHT_ROTATIONS),
        CLIMB(ClimberConstants.CLIMB_HEIGHT_ROTATIONS);

        private final double targetHeight;

        private ClimbingState(double targetHeight) {
            this.targetHeight = targetHeight;
        }
    }

    private final Climber climber;
    private ClimbingState climbState;

    public AutoStateClimb(Climber climber, ClimbingState state) {
        this.climber = climber;
    }

    @Override // TODO: Fix to drive to target height
    public void action(double periodSeconds) {
        if (!atHeight(climbState.targetHeight)) {
            climber.climb(climbState.targetHeight);
        }
    }

    private boolean atHeight(double height) {
        return climber.getPosition() >= height;
    }
}