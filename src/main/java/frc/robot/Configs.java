package frc.robot;

import frc.robot.Constants.SwerveConstants;
import frc.robot.Constants.ShooterConstants;

import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

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
                    .pid(0, 0, 0)
                    .outputRange(-1, 1).feedForward
                    .kS(SwerveConstants.FEED_FORWARD_DRIVE_STATIC)
                    .kV(SwerveConstants.FEED_FORWARD_DRIVE_VELOCITY);

            turningConfig
                    .idleMode(IdleMode.kBrake)
                    .smartCurrentLimit(20);
            turningConfig.encoder
                    .positionConversionFactor(SwerveConstants.ENCODER_TO_METERS_TURN_RATIO) // radians
                    .velocityConversionFactor(SwerveConstants.ENCODER_TO_METERS_TURN_RATIO / 60.0); // radians/second
            turningConfig.closedLoop
                    .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                    .positionWrappingEnabled(true)
                    .positionWrappingInputRange(0, 2 * Math.PI)
                    .pid(0.5, 0, 0.0)
                    .outputRange(-1, 1);
            // .feedForward
            // .kS(SwerveConstants.FEED_FORWARD_TURN_STATIC)
            // .kV(SwerveConstants.FEED_FORWARD_TURN_VELOCITY);
            // Enable PID wrap around for the turning motor. This will allow the PID
            // controller to go through 0 to get to the setpoint i.e. going from 350 degrees
            // to 10 degrees will go through 0 rather than the other direction which is a
            // longer route.
            turningConfig.closedLoop.maxMotion.allowedProfileError(0.04);
        }
    }

    public static final class Shooter {
        public static final SparkMaxConfig shooterConfig = new SparkMaxConfig();
        public static final SparkMaxConfig shooterFollowerConfig = new SparkMaxConfig();
        public static final SparkMaxConfig leftFeedConfig = new SparkMaxConfig();
        public static final SparkMaxConfig rightFeedConfig = new SparkMaxConfig();
        public static final SparkFlexConfig ventConfig = new SparkFlexConfig();

        static {
            shooterConfig
                    .inverted(true)
                    .idleMode(IdleMode.kCoast)
                    .smartCurrentLimit(40);
            shooterConfig.closedLoop
                    .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                    .pid(0.00001, 0, 0)
                    .outputRange(0, 1).feedForward
                    .kS(ShooterConstants.FEED_FORWARD_SHOOT_STATIC)
                    .kV(ShooterConstants.FEED_FORWARD_SHOOT_VELOCITY);
            // shooterConfig.closedLoop.maxMotion
            // .cruiseVelocity(5000)
            // .maxAcceleration(10000)
            // .allowedProfileError(1);

            leftFeedConfig
                    .inverted(false)
                    .idleMode(IdleMode.kBrake)
                    .openLoopRampRate(1.0)
                    .smartCurrentLimit(20);
            rightFeedConfig
                    .inverted(true)
                    .idleMode(IdleMode.kBrake)
                    .openLoopRampRate(1.0)
                    .smartCurrentLimit(20);
            ventConfig
                    .inverted(false)
                    .idleMode(IdleMode.kBrake)
                    .openLoopRampRate(1.0)
                    .smartCurrentLimit(20);
        }
    }

    public static final class Intake {
        public static final SparkMaxConfig deployConfig = new SparkMaxConfig();

        static {
            deployConfig
                    .inverted(false)
                    .idleMode(IdleMode.kBrake)
                    .openLoopRampRate(0.5)
                    .smartCurrentLimit(20);
        }
    }

    public static final class Climber {
        public static final SparkMaxConfig climberConfig = new SparkMaxConfig();

        static {
            climberConfig
                    .inverted(false)
                    .idleMode(IdleMode.kBrake)
                    .openLoopRampRate(0.5)
                    .smartCurrentLimit(20);
        }
    }
}
