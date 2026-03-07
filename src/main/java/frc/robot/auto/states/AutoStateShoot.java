package frc.robot.auto.states;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.subsystems.shooter.ShootSystem;
import frc.robot.subsystems.shooter.ShootSystem.ShooterState;

public class AutoStateShoot extends AbstractAutoState {
    ShootSystem shootSystem;
    double timeToShootSeconds;

    private double startTime;

    public AutoStateShoot(ShootSystem shootSystem, double timeToShootSeconds) {
        this.shootSystem = shootSystem;
        this.timeToShootSeconds = timeToShootSeconds;
    }

    @Override
    public void onStateEntry(double periodSeconds) {
        this.startTime = Timer.getFPGATimestamp();
    }

    @Override
    public void action(double periodSeconds) {
        shootSystem.setShooterState(ShooterState.SHOOT);
    }

    public boolean atTime(AbstractAutoState state) {
        return Timer.getFPGATimestamp() - startTime >= timeToShootSeconds;
    }

}
