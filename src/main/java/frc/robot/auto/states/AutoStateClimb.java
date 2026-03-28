package frc.robot.auto.states;

import frc.robot.Constants.ClimberConstants;
import frc.robot.subsystems.climber.Climber;

public class AutoStateClimb extends AbstractAutoState {

    private enum ClimbingState {
        REST(ClimberConstants.REST_HEIGHT_TICKS),
        SETUP(ClimberConstants.MAX_HEIGHT_TICKS),
        CLIMB(ClimberConstants.CLIMB_HEIGHT_TICKS);

        private final double targetHeight;

        private ClimbingState(double targetHeight) {
            this.targetHeight = targetHeight;
        }
    }

    private final Climber climber;
    private ClimbingState state;

    public AutoStateClimb(Climber climber, ClimbingState state) {
        this.climber = climber;
    }

    @Override // TODO: Fix to drive to target height
    public void action(double periodSeconds) {
        if (!atHeight(state.targetHeight)) {
            climber.climb(state.targetHeight);
        }
    }

    private boolean atHeight(double height) {
        return climber.getPosition() >= height;
    }
}