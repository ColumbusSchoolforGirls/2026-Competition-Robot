// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import frc.robot.Configs;
import frc.robot.Constants.SwerveConstants;

public class SwerveModule {
  public SparkMax driveMotor;
  public SparkMax turnMotor;

  public RelativeEncoder driveEncoder;
  public RelativeEncoder turnRelativeEncoder;
  // CTRE SRX Mag Encoder using pulses, used at RobotInit + buttonpressed to reset
  // relative encoder
  public DutyCycleEncoder turnAbsoluteEncoder;

  private SparkClosedLoopController driveClosedLoopController;
  private SparkClosedLoopController turnClosedLoopController;

  SparkMaxConfig config = new SparkMaxConfig();

  private double chassisAngularOffset; // This comes from constants

  /**
   * Constructs a SwerveModule with a drive motor, turning motor, drive encoder
   * and turning encoder.
   */
  public SwerveModule(int driveMotorID, int turnMotorID, int turnDIOPin, double chassisAngularOffset) {
    driveMotor = new SparkMax(driveMotorID, MotorType.kBrushless);
    turnMotor = new SparkMax(turnMotorID, MotorType.kBrushless);

    driveEncoder = driveMotor.getEncoder(); // Drive motor only has a relative encoder
    turnRelativeEncoder = turnMotor.getEncoder();
    turnAbsoluteEncoder = new DutyCycleEncoder(turnDIOPin);
    driveClosedLoopController = driveMotor.getClosedLoopController();
    turnClosedLoopController = turnMotor.getClosedLoopController();

    // Apply the respective configurations to the SPARKS. Reset parameters before
    // applying the configuration to bring the SPARK to a known good state. Persist
    // the settings to the SPARK to avoid losing them on a power cycle.

    driveMotor.configure(Configs.MAXSwerveModule.drivingConfig, ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);
    turnMotor.configure(Configs.MAXSwerveModule.turningConfig, ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);
    turnAbsoluteEncoder.setInverted(true);

    this.chassisAngularOffset = chassisAngularOffset;
    driveEncoder.setPosition(0);
    turnRelativeEncoder.setPosition(0);
  }

  public double getAbsoluteEncoderAngleRelativeToChassis() { // in radians
    return (turnAbsoluteEncoder.get() * 2 * Math.PI) - chassisAngularOffset;
  }

  public void updateModule() {
    turnRelativeEncoder.getVelocity();

    if (turnRelativeEncoder.getVelocity() < SwerveConstants.TURN_RESET_VELOCITY) {
      resetRelativeTurnEncoder();
    }
  }

  // Gets an angle in radians and normalizes it to be between 0 and 2pi
  public double normalizeAngle(double angle) {
    angle = angle % (2 * Math.PI);
    if (angle < 0) {
      angle += (2 * Math.PI);
    }
    return angle;
  }

  public void resetRelativeTurnEncoder() {
    double targetRelativeEncoder = getAbsoluteEncoderAngleRelativeToChassis();
    turnRelativeEncoder.setPosition(normalizeAngle(targetRelativeEncoder));
    SwerveModuleState desiredState = new SwerveModuleState(0.0, new Rotation2d());
    desiredState.angle = new Rotation2d(turnRelativeEncoder.getPosition());
    setDesiredState(desiredState);
  }

  // Returns the current state of the module.
  public SwerveModuleState getState() {
    return new SwerveModuleState(
        driveEncoder.getVelocity(), new Rotation2d(turnRelativeEncoder.getPosition()));
  }

  /** Returns the current position of the module. */
  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(
        driveEncoder.getPosition(), new Rotation2d(turnRelativeEncoder.getPosition()));
  }

  public double getDrivePositionMeters() {
    return driveEncoder.getPosition();
  }

  public double getVelocity() {
    return driveEncoder.getVelocity();
  }

  /**
   * Sets the desired state for the module.
   *
   * @param desiredState Desired state with speed and angle.
   */
  public void setDesiredState(SwerveModuleState desiredState) {
    SwerveModuleState correctedDesiredState = new SwerveModuleState();
    correctedDesiredState.speedMetersPerSecond = desiredState.speedMetersPerSecond;
    correctedDesiredState.angle = desiredState.angle;

    // Optimize the reference state to avoid spinning further than 90 degrees.
    Rotation2d encoderRotation = new Rotation2d(normalizeAngle(turnRelativeEncoder.getPosition()));
    correctedDesiredState.optimize(encoderRotation);

    // Scale speed by cosine of angle error. This scales down movement perpendicular
    // to the desired direction of travel that can occur when modules change
    // directions.
    // This results in smoother driving.
    correctedDesiredState.cosineScale(encoderRotation);

    driveClosedLoopController.setSetpoint(correctedDesiredState.speedMetersPerSecond, ControlType.kVelocity);
    turnClosedLoopController.setSetpoint(correctedDesiredState.angle.getRadians(), ControlType.kPosition);
  }

  /** Zeroes all the SwerveModule drive encoders. */
  public void resetEncoder() {
    driveEncoder.setPosition(0);
  }

  public void setBrakeMode() { // Should only run on init
    config.idleMode(SparkBaseConfig.IdleMode.kBrake);
    driveMotor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    turnMotor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
  }

  public void setCoastMode() { // Should only run on disable
    config.idleMode(SparkBaseConfig.IdleMode.kCoast);
    driveMotor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    turnMotor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
  }
}