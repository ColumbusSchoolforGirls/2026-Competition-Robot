package frc.robot.auto.states;

import frc.robot.subsystems.climber.Climber;
import frc.robot.Constants.ClimberConstants;

public class AutoStateClimb extends AbstractAutoState {
    private Climber climber;
    private double startingPosition;

    private enum ClimbingState {
        CLIMB,
    }

    public AutoStateClimb(Climber climber) {
        this.climber = climber;
    }

    @Override // TODO: Fix to drive to target height
    public void action(double periodSeconds, double targetHeightTicks) {
        climber.climb(targetHeightTicks);
    }

    private boolean atHeight(AutoStateClimb state) {
        return (Math.abs(climber.getPosition() - startingPosition) >= ClimberConstants.MAX_HEIGHT_TICKS);
    }
}