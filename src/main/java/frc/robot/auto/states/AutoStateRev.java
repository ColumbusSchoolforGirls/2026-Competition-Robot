package frc.robot.auto.states;

import frc.robot.subsystems.shooter.ShootSystem;
import frc.robot.subsystems.shooter.ShootSystem.ShooterState;

public class AutoStateRev extends AbstractAutoState {
    ShootSystem shootSystem;

    public AutoStateRev(ShootSystem shootSystem) {
        this.shootSystem = shootSystem;
    }

    @Override
    public void onStateEntry(double periodSeconds) {
        shootSystem.setShooterState(ShooterState.REV);
    }

    @Override
    public void action(double periodSeconds) {

    }
}