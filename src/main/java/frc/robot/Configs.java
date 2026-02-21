package frc.robot;

import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import frc.robot.Constants.SwerveConstants;

public class Configs {

    public static final class MAXSwerveModule {
        public static final SparkMaxConfig drivingConfig = new SparkMaxConfig();
        public static final SparkMaxConfig turningConfig = new SparkMaxConfig();

        static {
            // Use module constants to calculate conversion factors and feed forward gain.
            drivingConfig
                    .idleMode(IdleMode.kBrake)
                    .smartCurrentLimit(50);
            drivingConfig.encoder
                    .positionConversionFactor(SwerveConstants.ENCODER_TO_METERS_DRIVE_RATIO) // meters
                    .velocityConversionFactor(SwerveConstants.ENCODER_TO_METERS_DRIVE_RATIO / 60.0); // meters/second
            drivingConfig.closedLoop
                    .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                    // TODO: Adjust PID values for our robot
                    .pid(0.04, 0, 0)
                    .outputRange(-1, 1);

            turningConfig
                    .idleMode(IdleMode.kBrake)
                    .smartCurrentLimit(20);
            turningConfig.encoder
                    .positionConversionFactor(SwerveConstants.ENCODER_TO_METERS_TURN_RATIO) // radians
                    .velocityConversionFactor(SwerveConstants.ENCODER_TO_METERS_TURN_RATIO / 60.0); // radians/second
            turningConfig.closedLoop
                    .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                    .pid(0.5, 0, 0.0)
                    .outputRange(-1, 1)
                    // Enable PID wrap around for the turning motor. This will allow the PID
                    // controller to go through 0 to get to the setpoint i.e. going from 350 degrees
                    // to 10 degrees will go through 0 rather than the other direction which is a
                    // longer route.
                    .positionWrappingEnabled(true)
                    .positionWrappingInputRange(0, 2 * Math.PI);

            turningConfig.closedLoop.maxMotion.allowedProfileError(0.04);
        }
    }

    public static final class Shooter {
        public static final SparkMaxConfig shooterConfig = new SparkMaxConfig();
        public static final SparkMaxConfig shooterFollowerConfig = new SparkMaxConfig();
        public static final SparkMaxConfig feederConfig = new SparkMaxConfig();
        public static final SparkMaxConfig rollersConfig = new SparkMaxConfig();

        static {
            shooterConfig
                    .inverted(true)
                    .idleMode(IdleMode.kCoast)
                    .smartCurrentLimit(40);
            shooterConfig.closedLoop
                    .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                    // TODO: Adjust PID values for our robot
                    .pid(0.04, 0, 0)
                    .outputRange(-1, 1);
            shooterConfig.closedLoop.maxMotion // TODO: Test if MaxMotion is needed
                    .cruiseVelocity(5000)
                    .maxAcceleration(10000)
                    .allowedProfileError(1);

            feederConfig
                    .inverted(true) // TODO: Update depending on direction of feeder motors
                    .idleMode(IdleMode.kBrake)
                    .openLoopRampRate(1.0)
                    .smartCurrentLimit(20);

            rollersConfig
                    .inverted(true) // TODO: Update depending on direction of roller motors
                    .idleMode(IdleMode.kBrake)
                    .openLoopRampRate(0.5)
                    .smartCurrentLimit(20);
        }
    }
}
