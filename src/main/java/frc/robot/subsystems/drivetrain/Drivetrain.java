// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drivetrain;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;

import frc.robot.Constants;
import frc.robot.RobotSpecificConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.limelight.Limelight;

public class Drivetrain {

    private Limelight limelight;
    private double gyroDifference;
    private double targetAngle;
    private double driveDifference;
    private double targetDistance;
    private double startDriveTime;

    private final Translation2d frontLeftLocation;
    private final Translation2d frontRightLocation;
    private final Translation2d backLeftLocation;
    private final Translation2d backRightLocation;

    // Set up our absolute encoders.
    // For the 2026 robot, run with all CAN encoders, and Eva runs with
    // DutyCycleEncoders.
    private final AbsoluteEncoderInterface frontLeftAboluteEncoder;
    private final AbsoluteEncoderInterface backLeftAbsoluteEncoder;
    private final AbsoluteEncoderInterface frontRightAbsoluteEncoder;
    private final AbsoluteEncoderInterface backRightAbsoluteEncoder;

    private final SwerveModule frontLeft;
    private final SwerveModule backLeft;
    private final SwerveModule frontRight;
    private final SwerveModule backRight;

    private final AHRS gyro;

    private final SwerveDriveKinematics kinematics;

    private final SwerveDriveOdometry odometry;

    private double lastEncoderSyncTime = Timer.getFPGATimestamp();

    public Drivetrain(Limelight limelight) {
        this.limelight = limelight;
        gyro = new AHRS(NavXComType.kMXP_SPI);

        if (RobotSpecificConstants.ROBOT_NAME == RobotSpecificConstants.RobotName.BUBBLES) {
            frontLeftLocation = new Translation2d(RobotSpecificConstants.Unnamed2026Robot.TRANSLATION_2D_OFFSET,
                    -RobotSpecificConstants.Unnamed2026Robot.TRANSLATION_2D_OFFSET);
            frontRightLocation = new Translation2d(RobotSpecificConstants.Unnamed2026Robot.TRANSLATION_2D_OFFSET,
                    RobotSpecificConstants.Unnamed2026Robot.TRANSLATION_2D_OFFSET);
            backLeftLocation = new Translation2d(-RobotSpecificConstants.Unnamed2026Robot.TRANSLATION_2D_OFFSET,
                    -RobotSpecificConstants.Unnamed2026Robot.TRANSLATION_2D_OFFSET);
            backRightLocation = new Translation2d(-RobotSpecificConstants.Unnamed2026Robot.TRANSLATION_2D_OFFSET,
                    RobotSpecificConstants.Unnamed2026Robot.TRANSLATION_2D_OFFSET);

            frontLeftAboluteEncoder = new CANAbsoluteEncoder(RobotSpecificConstants.Unnamed2026Robot.FL_ENCODER_CAN_ID);
            backLeftAbsoluteEncoder = new CANAbsoluteEncoder(RobotSpecificConstants.Unnamed2026Robot.BL_ENCODER_CAN_ID);
            frontRightAbsoluteEncoder = new CANAbsoluteEncoder(
                    RobotSpecificConstants.Unnamed2026Robot.FR_ENCODER_CAN_ID);
            backRightAbsoluteEncoder = new CANAbsoluteEncoder(
                    RobotSpecificConstants.Unnamed2026Robot.BR_ENCODER_CAN_ID);

            frontLeft = new SwerveModule(DriveConstants.FL_DRIVE_ID, DriveConstants.FL_TURN_ID,
                    frontLeftAboluteEncoder, RobotSpecificConstants.Unnamed2026Robot.FL_CHASSIS_ANGULAR_OFFSET);
            backLeft = new SwerveModule(DriveConstants.BL_DRIVE_ID, DriveConstants.BL_TURN_ID,
                    backLeftAbsoluteEncoder, RobotSpecificConstants.Unnamed2026Robot.BL_CHASSIS_ANGULAR_OFFSET);
            frontRight = new SwerveModule(DriveConstants.FR_DRIVE_ID, DriveConstants.FR_TURN_ID,
                    frontRightAbsoluteEncoder, RobotSpecificConstants.Unnamed2026Robot.FR_CHASSIS_ANGULAR_OFFSET);
            backRight = new SwerveModule(DriveConstants.BR_DRIVE_ID, DriveConstants.BR_TURN_ID,
                    backRightAbsoluteEncoder, RobotSpecificConstants.Unnamed2026Robot.BR_CHASSIS_ANGULAR_OFFSET);
        } else {
            frontLeftLocation = new Translation2d(RobotSpecificConstants.EvaRobot.TRANSLATION_2D_OFFSET,
                    -RobotSpecificConstants.EvaRobot.TRANSLATION_2D_OFFSET);
            frontRightLocation = new Translation2d(RobotSpecificConstants.EvaRobot.TRANSLATION_2D_OFFSET,
                    RobotSpecificConstants.EvaRobot.TRANSLATION_2D_OFFSET);
            backLeftLocation = new Translation2d(-RobotSpecificConstants.EvaRobot.TRANSLATION_2D_OFFSET,
                    -RobotSpecificConstants.EvaRobot.TRANSLATION_2D_OFFSET);
            backRightLocation = new Translation2d(-RobotSpecificConstants.EvaRobot.TRANSLATION_2D_OFFSET,
                    RobotSpecificConstants.EvaRobot.TRANSLATION_2D_OFFSET);

            frontLeftAboluteEncoder = new DutyCycleAbsoluteEncoder(RobotSpecificConstants.EvaRobot.FL_DIO);
            backLeftAbsoluteEncoder = new CANAbsoluteEncoder(RobotSpecificConstants.EvaRobot.BL_DIO);
            frontRightAbsoluteEncoder = new CANAbsoluteEncoder(RobotSpecificConstants.EvaRobot.FL_DIO);
            backRightAbsoluteEncoder = new CANAbsoluteEncoder(RobotSpecificConstants.EvaRobot.BR_DIO);

            frontLeft = new SwerveModule(DriveConstants.FL_DRIVE_ID, DriveConstants.FL_TURN_ID,
                    frontLeftAboluteEncoder, RobotSpecificConstants.EvaRobot.FL_CHASSIS_ANGULAR_OFFSET);
            backLeft = new SwerveModule(DriveConstants.BL_DRIVE_ID, DriveConstants.BL_TURN_ID,
                    backLeftAbsoluteEncoder, RobotSpecificConstants.EvaRobot.BL_CHASSIS_ANGULAR_OFFSET);
            frontRight = new SwerveModule(DriveConstants.FR_DRIVE_ID, DriveConstants.FR_TURN_ID,
                    frontRightAbsoluteEncoder, RobotSpecificConstants.EvaRobot.FR_CHASSIS_ANGULAR_OFFSET);
            backRight = new SwerveModule(DriveConstants.BR_DRIVE_ID, DriveConstants.BR_TURN_ID,
                    backRightAbsoluteEncoder, RobotSpecificConstants.EvaRobot.BR_CHASSIS_ANGULAR_OFFSET);
        }

        kinematics = new SwerveDriveKinematics(
                frontLeftLocation, frontRightLocation, backLeftLocation, backRightLocation);
        odometry = new SwerveDriveOdometry(
                kinematics,
                getRotation2d(),
                new SwerveModulePosition[] {
                        frontLeft.getPosition(),
                        frontRight.getPosition(),
                        backLeft.getPosition(),
                        backRight.getPosition()
                });
    }

    public void robotInit() {
        zeroHeading();
        resetEncoders();
        resetRelativeTurnEncoders();
        setBrakeMode();
    }

    public void stageInit() {
        setBrakeMode();
        resetRelativeTurnEncoders();
    }

    public void disabledInit() {
        setCoastMode();
    }

    public double getDrivePositionMeters() {
        return frontRight.getDrivePositionMeters();
    }

    public void resetDistance() {
        resetEncoders();
    }

    /**
     * Method to drive the robot using joystick info.
     *
     * Using airplane coordinate system
     * 
     * @param xSpeed        Speed of the robot in the x direction (forward).
     * @param ySpeed        Speed of the robot in the y direction (sideways) --
     *                      Positive y to the left.
     * @param rot           Angular rate of the robot.
     * @param fieldRelative Whether the provided x and y speeds are relative to the
     *                      field.
     */
    public void drive(
            double xSpeed, double ySpeed, double rot, boolean fieldRelative, double periodSeconds) {
        SwerveModuleState[] swerveModuleStates = kinematics.toSwerveModuleStates(
                ChassisSpeeds.discretize(
                        fieldRelative
                                ? ChassisSpeeds.fromFieldRelativeSpeeds(
                                        xSpeed, ySpeed, rot, getRotation2d())
                                : new ChassisSpeeds(xSpeed, ySpeed, rot),
                        periodSeconds));
        setModuleStates(swerveModuleStates);
    }

    public void runTurn() {
        backLeft.getTurnMotor().set(0.3);
    }

    /**
     * Sets the swerve ModuleStates.
     *
     * @param desiredStates The desired SwerveModule states.
     */
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(
                desiredStates, DriveConstants.MAX_SPEED);
        frontLeft.setDesiredState(desiredStates[0]);
        frontRight.setDesiredState(desiredStates[1]);
        backLeft.setDesiredState(desiredStates[2]);
        backRight.setDesiredState(desiredStates[3]);
    }

    /** Returns the currently-estimated pose of the robot. */
    public Pose2d getPose() {
        return odometry.getPoseMeters();
    }

    /**
     * Resets the odometry to the specified pose.
     *
     * @param pose The pose to which to set the odometry.
     */
    public void resetOdometry(Pose2d pose) {
        odometry.resetPosition(
                getRotation2d(),
                new SwerveModulePosition[] {
                        frontLeft.getPosition(),
                        frontRight.getPosition(),
                        backLeft.getPosition(),
                        backRight.getPosition()
                },
                pose);
    }

    /** Update the odometry in the periodic block. */
    public void periodic() {
        odometry.update(
                getRotation2d(),
                new SwerveModulePosition[] {
                        frontLeft.getPosition(),
                        frontRight.getPosition(),
                        backLeft.getPosition(),
                        backRight.getPosition()
                });
        updateDashboard();
        // timedResetRelativeTurnEncoders();
    }

    public void resetEncoders() {
        frontLeft.resetEncoder();
        frontRight.resetEncoder();
        backLeft.resetEncoder();
        backRight.resetEncoder();
    }

    public void resetRelativeTurnEncoders() {
        frontLeft.resetRelativeTurnEncoder();
        frontRight.resetRelativeTurnEncoder();
        backLeft.resetRelativeTurnEncoder();
        backRight.resetRelativeTurnEncoder();
    }

    public void timedResetRelativeTurnEncoders() {
        double currentTime = Timer.getFPGATimestamp();
        if (currentTime - lastEncoderSyncTime > 5.0) {
            lastEncoderSyncTime = currentTime;
            resetRelativeTurnEncoders();
        }
    }

    public void zeroHeading() {
        gyro.reset();
    }

    /** Returns the heading of the robot in degrees from -180 to 180. */
    public double getHeading() {
        return gyro.getYaw() * (DriveConstants.GYRO_REVERSED ? -1.0 : 1.0);
    }

    /** Returns the turn rate of the robot in degrees per second. */
    public double getTurnRate() {
        return gyro.getRate() * (DriveConstants.GYRO_REVERSED ? -1.0 : 1.0);
    }

    public Rotation2d getRotation2d() {
        double deg = gyro.getAngle();
        if (DriveConstants.GYRO_REVERSED) {
            deg = -deg;
        }
        return Rotation2d.fromDegrees(deg);
    }

    // This is for auto turning
    public void setAutoTargetAngle(double targetAngle) {
        this.targetAngle = targetAngle;
    }

    public boolean turnComplete() {
        gyroDifference = (getHeading() - targetAngle);

        return Math.abs(gyroDifference) < Constants.DriveConstants.TURN_TOLERANCE;
    }

    public void autoAlign(double periodSeconds) {
        final var rot_limelight = limelight.limelight_aim_proportional();
        final var forward_limelight = limelight.limelight_range_proportional();

        // while using Limelight, turn off field-relative driving.
        boolean fieldRelative = false;

        drive(forward_limelight, 0.0, rot_limelight, fieldRelative, periodSeconds);
    }

    public void startDrive(double distanceMeters) {
        resetEncoders();
        targetDistance = distanceMeters;
        startDriveTime = Timer.getFPGATimestamp();
    }

    public void startTurn(double angle) {
        zeroHeading();
        this.targetAngle = (angle + getHeading());
    }

    public void stop(double periodSeconds) {
        drive(0, 0, 0, false, periodSeconds);
    }

    public void resetGyro() {
        gyro.reset();
    }

    public void gyroTurn(double periodSeconds) {
        gyroDifference = (getHeading() - targetAngle);

        if (Math.abs(gyroDifference) < Constants.DriveConstants.TURN_TOLERANCE) {
            drive(0, 0, 0, true, periodSeconds);
        } else if (gyroDifference < 0) {
            drive(0, 0, 0.05 * Math.abs(gyroDifference) + 0.2, true, periodSeconds);
        } else if (gyroDifference > 0) {
            drive(0, 0, -0.05 * gyroDifference - 0.2, true, periodSeconds);
        }
    }

    private void updateDashboard() {
        SmartDashboard.putNumber("FL DriveEncoder",
                frontLeft.getDrivePositionMeters());
        SmartDashboard.putNumber("FR DriveEncoder",
                frontRight.getDrivePositionMeters());
        SmartDashboard.putNumber("BL DriveEncoder",
                backLeft.getDrivePositionMeters());
        SmartDashboard.putNumber("BR DriveEncoder",
                backRight.getDrivePositionMeters());

        SmartDashboard.putNumber("FL AbsEncoder", frontLeft.turnAbsoluteEncoder.get()
                * 360);
        SmartDashboard.putNumber("FR AbsEncoder",
                frontRight.turnAbsoluteEncoder.get() * 360);
        SmartDashboard.putNumber("BL AbsEncoder", backLeft.turnAbsoluteEncoder.get()
                * 360);
        SmartDashboard.putNumber("BR AbsEncoder", backRight.turnAbsoluteEncoder.get()
                * 360);

        SmartDashboard.putNumber("FL RelEncoder",
                (frontLeft.turnRelativeEncoder.getPosition() * 180) / Math.PI);
        SmartDashboard.putNumber("FR RelEncoder",
                (frontRight.turnRelativeEncoder.getPosition() * 180) / Math.PI);
        SmartDashboard.putNumber("BL RelEncoder",
                (backLeft.turnRelativeEncoder.getPosition() * 180) / Math.PI);
        SmartDashboard.putNumber("BR RelEncoder",
                (backRight.turnRelativeEncoder.getPosition() * 180) / Math.PI);

        SmartDashboard.putNumber("Gyro Angle", getHeading());
        SmartDashboard.putNumber("Gyro Yaw", gyro.getYaw());

        SmartDashboard.putNumber("FL Power",
                frontLeft.getDriveMotor().getAppliedOutput());
        SmartDashboard.putNumber("FR Power",
                frontRight.getDriveMotor().getAppliedOutput());
        SmartDashboard.putNumber("BL Power",
                backLeft.getDriveMotor().getAppliedOutput());
        SmartDashboard.putNumber("BR Power",
                backRight.getDriveMotor().getAppliedOutput());

        SmartDashboard.putNumber("FL Turn Power",
                frontLeft.getTurnMotor().getAppliedOutput());
        SmartDashboard.putNumber("FR Turn Power",
                frontRight.getTurnMotor().getAppliedOutput());
        SmartDashboard.putNumber("BL Turn Power",
                backLeft.getTurnMotor().getAppliedOutput());
        SmartDashboard.putNumber("BR Turn Power",
                backRight.getTurnMotor().getAppliedOutput());

        SmartDashboard.putNumber("FL Velocity",
                frontLeft.getDriveMotor().getEncoder().getVelocity());
        SmartDashboard.putNumber("FR Velocity",
                frontRight.getDriveMotor().getEncoder().getVelocity());
        SmartDashboard.putNumber("BL Velocity",
                backLeft.getDriveMotor().getEncoder().getVelocity());
        SmartDashboard.putNumber("BR Velocity",
                backRight.getDriveMotor().getEncoder().getVelocity());
        SmartDashboard.putNumber("FL Voltage",
                frontLeft.getDriveMotor().getAppliedOutput() *
                        frontLeft.getDriveMotor().getBusVoltage());
        SmartDashboard.putNumber("FR Voltage",
                frontRight.getDriveMotor().getAppliedOutput() *
                        frontRight.getDriveMotor().getBusVoltage());
        SmartDashboard.putNumber("BL Voltage",
                backLeft.getDriveMotor().getAppliedOutput() *
                        backLeft.getDriveMotor().getBusVoltage());
        SmartDashboard.putNumber("BR Voltage",
                backRight.getDriveMotor().getAppliedOutput() *
                        backRight.getDriveMotor().getBusVoltage());

        SmartDashboard.putNumber("FL Turn Voltage",
                frontLeft.getTurnMotor().getAppliedOutput() * frontLeft.getTurnMotor().getBusVoltage());
        SmartDashboard.putNumber("FR Turn Voltage",
                frontRight.getTurnMotor().getAppliedOutput() * frontRight.getTurnMotor().getBusVoltage());
        SmartDashboard.putNumber("BL Turn Voltage",
                backLeft.getTurnMotor().getAppliedOutput() * backLeft.getTurnMotor().getBusVoltage());
        SmartDashboard.putNumber("BR Turn Voltage",
                backRight.getTurnMotor().getAppliedOutput() * backRight.getTurnMotor().getBusVoltage());
        SmartDashboard.putNumber("FL Turn Velocity",
                frontLeft.getTurnMotor().getEncoder().getVelocity());
        SmartDashboard.putNumber("FR Turn Velocity",
                frontRight.getTurnMotor().getEncoder().getVelocity());
        SmartDashboard.putNumber("BL Turn Velocity",
                backLeft.getTurnMotor().getEncoder().getVelocity());
        SmartDashboard.putNumber("BR Turn Velocity",
                backRight.getTurnMotor().getEncoder().getVelocity());
    }

    private void setBrakeMode() {
        frontLeft.setBrakeMode();
        frontRight.setBrakeMode();
        backLeft.setBrakeMode();
        backRight.setBrakeMode();
    }

    private void setCoastMode() {
        frontLeft.setCoastMode();
        frontRight.setCoastMode();
        backLeft.setCoastMode();
        backRight.setCoastMode();
    }

}