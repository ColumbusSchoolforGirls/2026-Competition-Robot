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

        public static final int FR_DRIVE_ID = 4;
        public static final int FR_TURN_ID = 5;

        public static final int BL_DRIVE_ID = 6;
        public static final int BL_TURN_ID = 7;

        public static final int BR_DRIVE_ID = 8;
        public static final int BR_TURN_ID = 9;

        public static final int TURN_TOLERANCE = 2;
        public static final double DISTANCE_TOLERANCE = 0.05; // meters

        // Gyro
        public static final boolean GYRO_REVERSED = false; // TODO: Maybe doesn't affect anything
        public static final double STALL_SPEED = 0.1;

        public static final double TX_TOLERANCE = 2.0;
        public static final double TY_TOLERANCE = 2.0;
        public static final double TARGET_TA_VALUE = 3.40;

        public static final double METERS_TO_INCHES = 39.37; // found by inches/meter

        public static final double CRAWL_SPEED = 0.5;

        public static final double MAX_DRIVE_AUTO_TIME = 3.0;

        // Limelight
        public static final double NO_TX = 0;
        public static final double NO_TY = 0;

        public static final double ALIGN_TOLERANCE = 0.5; // TODO: TUNE, MADE UP VALUE
    }

    public static final class ControllerConstants {
        public static final double TRIGGER_DEADZONE = 0.1;
        public static final double JOYSTICK_DEADZONE = 0.1;
    }

    public static final class ShooterConstants {
        // The lead motor is the inverted motor (right of the shooter module).
        public static final int LEFT_LEAD_ID = 30;
        public static final int LEFT_FOLLOWER_ID = 31;
        public static final int RIGHT_LEAD_ID = 40;
        public static final int RIGHT_FOLLOWER_ID = 41;

        public static final int LEFT_FEEDER_ID = 8;
        public static final int RIGHT_FEEDER_ID = 9;

        public static final int VENT_ID = 21;

        public static final int SHOOT_RPM = 3700;
        public static final double FEEDER_PERCENTAGE_OUTPUT = 0.5;
        public static final double VENT_PERCENTAGE_OUTPUT = 0.5;

        public static final int RPM_TOLERANCE = 100; // TODO: Update, hopefully lower, after tuning the PID
    }

    public static final class IntakeConstants {
        public static final int DEPLOY_ID = 20;
        public static final int ROLLER_ID = 22;
        public static final int DEPLOYED_LIMIT_SWITCH_CHANNEL = 7; // TODO: Update with DIO channel
        public static final int RETRACTED_LIMIT_SWITCH_CHANNEL = 8;

        public static final double DEPLOY_PERCENTAGE_OUTPUT = 0.1;
        public static final double RETRACT_PERCENTAGE_OUTPUT = -1;
        public static final double ROLLER_PERCENTAGE_OUTPUT = -0.5;

        public static final double DEPLOYED_TICK_DISTANCE = 840;
    }

    public static final class HopperConstants {
        public static final int HOPPER_ID = 23;
        public static final double HOPPER_PERCENTAGE_OUTPUT = 0.5;
    }

    public static final class ClimberConstants {
        public static final int CLIMBER_ID = 50;

        private static final double PULLEY_CIRCUMFERENCE_INCHES = 0.958 * Math.PI;
        private static final double MAX_HEIGHT_INCHES = 8.5;
        private static final double CLIMB_HEIGHT_INCHES = 3.5;
        public static final double MAX_HEIGHT_TICKS = MAX_HEIGHT_INCHES / PULLEY_CIRCUMFERENCE_INCHES * 2688;
        public static final double CLIMB_HEIGHT_TICKS = CLIMB_HEIGHT_INCHES / PULLEY_CIRCUMFERENCE_INCHES * 2688;
        public static final double REST_HEIGHT_TICKS = 0;
        public static final double CLIMB_TICKS_TOLERANCE = 30; // TODO: TUNE

        // 64:1 Motor:Pulley ratio
        // 0.958 in. diameter pulley
        // 42 ticks/rev (NEO) * 64:1 ratio = 2,688 motor ticks/rev pulley
        // 8.5 inches up to setup, 5 inches down to climb
    }

    public static final class AutoConstants {
        public static final double LEAVE_ONLY_DISTANCE = 0.0;
    }
}