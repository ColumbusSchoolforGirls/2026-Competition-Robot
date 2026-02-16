package frc.robot;

public class Constants {

    public static final class SwerveConstants { // TODO: Update all chassis related constants for new chassis
        public static final int STATIC_GAIN_DRIVE = 1;
        public static final int VELOCITY_GAIN_DRIVE = 3;
        public static final int STATIC_GAIN_TURN = 1;
        public static final double VELOCITY_GAIN_TURN = 0.5;

        // The MAXSwerve module can be configured with one of three pinion gears: 12T,
        // 13T, or 14T. This changes the drive speed of the module (a pinion gear with
        // more teeth will result in a robot that drives faster).
        public static final int DRIVING_MOTOR_PINION_TEETH = 14;

        public static final double FREE_SPEED_RPM = 5676;

        // Calculations required for driving motor conversion factors and feed forward
        public static final double DRIVING_MOTOR_FREE_SPEED_RPS = FREE_SPEED_RPM / 60;
        public static final double WHEEL_DIAMETER_METERS = 0.0762;
        public static final double WHEEL_CIRCUMFERENCE_METERS = WHEEL_DIAMETER_METERS * Math.PI;
        // 45 teeth on the wheel's bevel gear, 22 teeth on the first-stage spur gear, 15
        // teeth on the bevel pinion
        public static final double DRIVING_MOTOR_REDUCTION = (45.0 * 22) / (DRIVING_MOTOR_PINION_TEETH * 15);
        public static final double DRIVE_WHEEL_FREE_SPEED_RPS = (DRIVING_MOTOR_FREE_SPEED_RPS
                * WHEEL_CIRCUMFERENCE_METERS)
                / DRIVING_MOTOR_REDUCTION;

        // Encoder calculations
        public static final double GEAR_RATIO = 8.14;
        public static final double WHEEL_DIAMETER = 4 / Math.PI; // in inches

        public static final double ENCODER_TO_METERS_DRIVE_RATIO = Constants.SwerveConstants.WHEEL_DIAMETER_METERS
                * Math.PI
                / Constants.SwerveConstants.DRIVING_MOTOR_REDUCTION;
        public static final double STEERING_FACTOR = 150 / 7;
        // Converts encoder to radians,
        public static final double ENCODER_TO_METERS_TURN_RATIO = (2 * Math.PI) / STEERING_FACTOR;
        public static final double DRIVING_VELOCITY_FEED_FORWARD = 1
                / Constants.SwerveConstants.DRIVE_WHEEL_FREE_SPEED_RPS;

        public static final double TURN_RESET_VELOCITY = 1;
    }

    public static final class DriveConstants {

        public static final double MAX_SPEED = 3.0; // 3 meters per second
        public static final double MAX_ANGULAR_SPEED = Math.PI * 2; // 1 rotation per second

        public static final double TRANSLATION_2D_OFFSET = 0.3048; // 12 inches to meters

        // NOTE: All angles must be thought of as clockwise negative (-) because of the
        // gear-pulley system.
        // All angular offsets are in angles (radians -- *Math.PI/180).
        public static final int FL_DRIVE_ID = 2;
        public static final int FL_TURN_ID = 3;
        public static final int FL_DIO = 8;
        public static final double FL_CHASSIS_ANGULAR_OFFSET = (307.4 * Math.PI) / 180; // Past value: 52.7

        public static final int FR_DRIVE_ID = 4;
        public static final int FR_TURN_ID = 5;
        public static final int FR_DIO = 7;
        public static final double FR_CHASSIS_ANGULAR_OFFSET = (358.3 * Math.PI) / 180; // Past value: 3.4

        public static final int BL_DRIVE_ID = 6;
        public static final int BL_TURN_ID = 7;
        public static final int BL_DIO = 9;
        public static final double BL_CHASSIS_ANGULAR_OFFSET = (278.1 * Math.PI) / 180; // Past value: 85

        public static final int BR_DRIVE_ID = 8;
        public static final int BR_TURN_ID = 9;
        public static final int BR_DIO = 6;
        public static final double BR_CHASSIS_ANGULAR_OFFSET = (177.4 * Math.PI / 180); // Past value: -178

        public static final int TURN_TOLERANCE = 2;
        public static final double DISTANCE_TOLERANCE = 0.02;

        // Gyro
        public static final boolean GYRO_REVERSED = false;
        public static final double STALL_SPEED = 0.1;

        public static final double TX_TOLERANCE = 2.0;
        public static final double TY_TOLERANCE = 2.0;
        public static final double TARGET_TA_VALUE = 3.40;

        public static final double METERS_TO_INCHES = 39.37; // found by inches/meter

        public static final double CRAWL_SPEED = 0.5;

        public static final double MAX_DRIVE_AUTO_TIME = 3.0;

    }

    public static final class ControllerConstants {

        public static final int REVERSE_CLIMBER_AUX_PORT = 2;

        public static final double TRIGGER_DEADZONE = 0.1;
        public static final double JOYSTICK_DEADZONE = 0.1;
    }

    public static final class ShooterConstants {
        // The lead motor is the inverted motor (right of the shooter module).
        public static final int LEFT_LEAD_ID = 100; // TODO: Update motor IDs
        public static final int LEFT_FOLLOWER_ID = 101;
        public static final int RIGHT_LEAD_ID = 102;
        public static final int RIGHT_FOLLOWER_ID = 103;

        public static final int RIGHT_FEEDER_ID = 104;
        public static final int LEFT_FEEDER_ID = 105;

        public static final int ROLLERS_ID = 106;

        public static final int SHOOT_SPEED = 1000;
        public static final double FEEDER_SPEED = 0.5;
    }
}