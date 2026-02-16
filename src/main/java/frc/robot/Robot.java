// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Limelight;

public class Robot extends TimedRobot {
  private final Limelight limelight = new Limelight("limelight-coral");
  private final Drivetrain drivetrain = new Drivetrain();
  private final JoystickControls joystickControls = new JoystickControls(drivetrain, limelight);

  @Override
  public void robotInit() {

    // Record both DS control and joystick data
    drivetrain.driveInit();
  }

  @Override
  public void robotPeriodic() {
    limelight.updateShuffleboardLimelightValues();
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

    // isFieldRelative(); //TODO: To implement all field-relative functions/driving
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
