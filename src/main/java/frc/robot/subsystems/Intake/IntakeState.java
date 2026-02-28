package frc.robot.subsystems.Intake;

public enum IntakeState {
    IN {
        public IntakeState nextState() {
            return DEPLOYING;
        }
    }
    , DEPLOYING, OUT, RETRACTING
}
