// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import frc.robot.auto.AutoStateMachine;
import frc.robot.auto.states.AbstractAutoState;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.shooter.ShootSystem;
import frc.robot.subsystems.hopper.Hopper;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.limelight.Limelight;

public class Robot extends TimedRobot {
    private final Limelight limelightShoot = new Limelight("limelight-shoot");
    private final Drivetrain drivetrain = new Drivetrain();
    private final ShootSystem shootSystem = new ShootSystem();
    private final Intake intake = new Intake();
    private final Hopper hopper = new Hopper();
    private final Climber climber = new Climber();
    private final JoystickControls joystickControls = new JoystickControls(drivetrain, limelightShoot, shootSystem,
            intake, hopper, climber);
    private final AutoStateMachine autoStateMachine = new AutoStateMachine(drivetrain, limelightShoot, shootSystem);

    @Override
    public void robotInit() {
        drivetrain.driveInit();
        climber.robotInit();
        intake.robotInit();
        shootSystem.init();
    }

    @Override
    public void robotPeriodic() {
        limelightShoot.updateLimelightDashboard();
        drivetrain.periodic();
        drivetrain.updateDashboard();
        shootSystem.updateDashboard();
        climber.updateDashboard();
        intake.updateDashboard();

    }

    @Override
    public void autonomousInit() {
        intake.stageInit();
        drivetrain.setBrakeMode();
        drivetrain.resetRelativeTurnEncoders();
        shootSystem.init();
        joystickControls.init();
    }

    @Override
    public void autonomousPeriodic() {
        AbstractAutoState startState = autoStateMachine.buildPath();
        autoStateMachine.runStateMachine(startState, getPeriod());
    }

    @Override
    public void teleopInit() {
        intake.stageInit();
        shootSystem.init();
        drivetrain.setBrakeMode();
        drivetrain.resetRelativeTurnEncoders();
        joystickControls.init();
    }

    @Override
    public void teleopPeriodic() {
        joystickControls.driveWithJoystick(getPeriod());
        joystickControls.driverResetTurnEncoders();
        joystickControls.shoot();
        joystickControls.intake();
        joystickControls.hopper();
        joystickControls.climber();
        intake.periodic();
    }

    /** This function is called once when the robot is disabled. */
    @Override
    public void disabledInit() {
        drivetrain.setCoastMode();
    }

    /** This function is called periodically when disabled. */
    @Override
    public void disabledPeriodic() {
    }

    /** This function is called once when test mode is enabled. */
    @Override
    public void testInit() {
    }

    /** This function is called periodically during test mode. */
    @Override
    public void testPeriodic() {
    }

    /** This function is called once when the robot is first started up. */
    @Override
    public void simulationInit() {
    }

    /** This function is called periodically whilst in simulation. */
    @Override
    public void simulationPeriodic() {
    }
}
