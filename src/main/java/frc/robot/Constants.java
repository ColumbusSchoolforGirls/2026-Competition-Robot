package frc.robot;

import edu.wpi.first.wpilibj.XboxController;

public class Constants {

    public static final class SwerveConstants {
        // feed forward constants //TODO: change values when implementing feedforward
        public static final int STATIC_GAIN_DRIVE = 1;
        public static final int VELOCITY_GAIN_DRIVE = 3;
        public static final int STATIC_GAIN_TURN = 1;
        public static final double VELOCITY_GAIN_TURN = 0.5;

        // The MAXSwerve module can be configured with one of three pinion gears: 12T,
        // 13T, or 14T. This changes the drive speed of the module (a pinion gear with
        // more teeth will result in a robot that drives faster).
        public static final int DrivingMotorPinionTeeth = 14;

        public static final double FreeSpeedRpm = 5676;

        // Calculations required for driving motor conversion factors and feed forward
        public static final double DrivingMotorFreeSpeedRps = FreeSpeedRpm / 60;
        public static final double WheelDiameterMeters = 0.0762;
        public static final double WheelCircumferenceMeters = WheelDiameterMeters * Math.PI;
        // 45 teeth on the wheel's bevel gear, 22 teeth on the first-stage spur gear, 15
        // teeth on the bevel pinion
        public static final double DrivingMotorReduction = (45.0 * 22) / (DrivingMotorPinionTeeth * 15);
        public static final double DriveWheelFreeSpeedRps = (DrivingMotorFreeSpeedRps * WheelCircumferenceMeters)
                / DrivingMotorReduction;

        // Encoder calculations
        public static final double GEAR_RATIO = 8.14;
        public static final double WHEEL_DIAMETER = 4 / Math.PI; // in inches

        public static final double drivingFactor = Constants.SwerveConstants.WheelDiameterMeters * Math.PI
                / Constants.SwerveConstants.DrivingMotorReduction;
        public static final double steeringFactor = 150 / 7;
        public static final double turningFactor = (2 * Math.PI) / steeringFactor;
        public static final double drivingVelocityFeedForward = 1 / Constants.SwerveConstants.DriveWheelFreeSpeedRps;

        public static final double TURN_RESET_VELOCITY = 1;
    }

    public static final class DriveConstants {

        public static final double MAX_SPEED = 3.0; // 3 meters per second
        public static final double MAX_ANGULAR_SPEED = Math.PI * 2; // 1 rotation per second

        public static final double TRANSLATION_2D_OFFSET = 0.3048; // 12 inches to meters

        // NOTE: All angles must be thought of as clockwise negative (-) because of the gear-pulley system.
        // All angular offsets are in angles (radians -- *Math.PI/180).
        public static final int FL_DRIVE_ID = 2;
        public static final int FL_TURN_ID = 3;
        public static final int FL_DIO = 8;
        public static final double FL_CHASSIS_ANGULAR_OFFSET = (127.4 * Math.PI) / 180;  //Past value: 52.7

        public static final int FR_DRIVE_ID = 4;
        public static final int FR_TURN_ID = 5;
        public static final int FR_DIO = 7;
        public static final double FR_CHASSIS_ANGULAR_OFFSET = (358.3 * Math.PI) / 180; //Past value: 3.4

        public static final int BL_DRIVE_ID = 6;
        public static final int BL_TURN_ID = 7;
        public static final int BL_DIO = 9;
        public static final double BL_CHASSIS_ANGULAR_OFFSET = (98.1 * Math.PI) / 180; //Past value: 85

        public static final int BR_DRIVE_ID = 8;
        public static final int BR_TURN_ID = 9;
        public static final int BR_DIO = 6;
        public static final double BR_CHASSIS_ANGULAR_OFFSET = (177.4 * Math.PI / 180); //Past value: -178

        public static final int TURN_TOLERANCE = 2; // TODO: test this tolerance
        public static final double DISTANCE_TOLERANCE = 0.02; // TODO: test this tolerance

        // Gyro
        public static final boolean GyroReversed = false; //Possibly always positive
        public static final double STALL_SPEED = 0.1;

        public static final double TX_TOLERANCE = 2.0;
        public static final double TY_TOLERANCE = 2.0;
        public static final double TARGET_TA_VALUE = 3.40;

        public static final double METERS_TO_INCHES = 39.37; //found by inches/meter

        public static final double CRAWL_SPEED = 0.5;

        public static final double MAX_DRIVE_AUTO_TIME = 3.0;

    }

    public static final class AutoConstants {
        public static final double INITIAL_DISTANCE = 58.5; // inches
        public static final double LEAVE_ONLY_DISTANCE = 1.0;//10.0/DriveConstants.METERS_TO_INCHES; // inches

    }
    public static final class ControllerConstants {

        public static final XboxController DRIVE_CONTROLLER = new XboxController(0);
        public static final XboxController AUX = new XboxController(1);

        public static final int REVERSE_CLIMBER_AUX_PORT = 2;

        public static final double TRIGGER_DEADZONE = 0.1;
    }
}