// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import frc.robot.auto.AutoStateMachine;
import frc.robot.auto.states.AbstractAutoState;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.shooter.ShootSystem;
import frc.robot.subsystems.hopper.Hopper;
import frc.robot.subsystems.intake.Intake;

public class Robot extends TimedRobot {
    private final Limelight limelightShoot = new Limelight("limelight-shoot");
    private final Limelight limelightClimb = new Limelight("limelight-climb");
    private final Drivetrain drivetrain = new Drivetrain();
    private final ShootSystem shootSystem = new ShootSystem();
    private final Intake intake = new Intake();
    private final Hopper hopper = new Hopper();
    private final JoystickControls joystickControls = new JoystickControls(drivetrain, limelightShoot, shootSystem,
            intake, hopper);
    private final AutoStateMachine autoStateMachine = new AutoStateMachine(drivetrain, limelightShoot);

    @Override
    public void robotInit() {

        // Record both DS control and joystick data
        drivetrain.driveInit();
    }

    @Override
    public void robotPeriodic() {
        limelightShoot.updateShuffleboardLimelightValues();
        drivetrain.updateSmartDashboard();
        drivetrain.periodic();
    }

    @Override
    public void autonomousInit() {
        drivetrain.setBrakeMode();
        drivetrain.resetRelativeTurnEncoders();
    }

    @Override
    public void autonomousPeriodic() {
        AbstractAutoState startState = autoStateMachine.buildPath();
        autoStateMachine.runStateMachine(startState, getPeriod());
    }

    @Override
    public void teleopInit() {
        drivetrain.setBrakeMode();
        drivetrain.resetRelativeTurnEncoders();
    }

    @Override
    public void teleopPeriodic() {
        joystickControls.driveWithJoystick(getPeriod());
        joystickControls.driverResetTurnEncoders();
        joystickControls.shoot();
        joystickControls.intake();
        joystickControls.hopper();
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
