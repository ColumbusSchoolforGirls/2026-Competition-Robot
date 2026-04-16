package frc.robot.auto.states;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.subsystems.shooter.ShootSystem;
import frc.robot.subsystems.hopper.Hopper;
import frc.robot.subsystems.shooter.ShootSystem.ShooterState;

public class AutoStateShoot extends AbstractAutoState {
    ShootSystem shootSystem;
    Hopper hopper;
    double timeToShootSeconds;

    private double startTime;

    public AutoStateShoot(ShootSystem shootSystem, Hopper hopper, double timeToShootSeconds) {
        this.shootSystem = shootSystem;
        this.hopper = hopper;
        this.timeToShootSeconds = timeToShootSeconds;
    }

    @Override
    public void onStateEntry(double periodSeconds) {
        this.startTime = Timer.getFPGATimestamp();
        hopper.runHopper(true);
    }

    @Override
    public void onStateExit(double periodSeconds) {
        shootSystem.setShooterState(ShooterState.STOPPED);
        hopper.runHopper(false);
        hopper.setHopper();
    }

    @Override
    public void action(double periodSeconds) {
        shootSystem.setShooterState(ShooterState.SHOOT);
        hopper.setHopper();
    }

    public boolean atTime(AbstractAutoState state) {
        return Timer.getFPGATimestamp() - startTime >= timeToShootSeconds;
    }
}
