package frc.robot;

public class RobotSpecificConstants {
    public enum RobotName {
        EVA,
        BUBBLES
    }

    public static final RobotName ROBOT_NAME = RobotName.BUBBLES;

    public static final class EvaRobot {
        public static final int FL_DIO = 2;
        public static final double FL_CHASSIS_ANGULAR_OFFSET = (307.4 * Math.PI) / 180;

        public static final int FR_DIO = 3;
        public static final double FR_CHASSIS_ANGULAR_OFFSET = (358.3 * Math.PI) / 180;

        public static final int BL_DIO = 6; // For DIO drives
        public static final double BL_CHASSIS_ANGULAR_OFFSET = (278.1 * Math.PI) / 180;

        public static final int BR_DIO = 1;
        public static final double BR_CHASSIS_ANGULAR_OFFSET = (177.4 * Math.PI / 180);

        public static final double TRANSLATION_2D_OFFSET = 0.3048; // 12 inches to meters
    }

    public static final class Unnamed2026Robot {
        public static final int FL_ENCODER_CAN_ID = 10;
        public static final double FL_CHASSIS_ANGULAR_OFFSET = (-151.875 * Math.PI) / 180;

        public static final int FR_ENCODER_CAN_ID = 11;
        public static final double FR_CHASSIS_ANGULAR_OFFSET = (-81.125 * Math.PI) / 180;

        public static final int BL_ENCODER_CAN_ID = 12; // for Cancoder drives
        public static final double BL_CHASSIS_ANGULAR_OFFSET = (-136.04 * Math.PI) / 180;

        public static final int BR_ENCODER_CAN_ID = 13;
        public static final double BR_CHASSIS_ANGULAR_OFFSET = (39.20 * Math.PI / 180);

        public static final double TRANSLATION_2D_OFFSET = 0.2794; // 11 inches to meters
    }

}
