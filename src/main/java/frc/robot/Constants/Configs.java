package frc.robot.Constants;

import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.FeedForwardConfig;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

public final class Configs {
    public static final class SwerveModule {
        public static final SparkMaxConfig TURNING_CONFIG = new SparkMaxConfig();
        public static final SparkFlexConfig FL_CONFIG = new SparkFlexConfig();
        public static final SparkFlexConfig FR_CONFIG = new SparkFlexConfig();
        public static final SparkFlexConfig BL_CONFIG = new SparkFlexConfig();
        public static final SparkFlexConfig BR_CONFIG = new SparkFlexConfig();
      
        static {
            //Module constants used to calculate conversion factors and feed forward gain.
            double DRIVING_FACTOR = Drive.ModuleConstants.WHEEL_DIAMETER * Math.PI
                / 3.56;
            double FF_VELOCITY = 1 / Drive.ModuleConstants.DRIVE_WHEEL_FREE_RPS;
            double TURNING_FACTOR = 2 * Math.PI;

            TURNING_CONFIG
                .idleMode(IdleMode.kBrake)
                
                .smartCurrentLimit(20);
            TURNING_CONFIG.absoluteEncoder
                .inverted(true)
                .positionConversionFactor(TURNING_FACTOR)
                .velocityConversionFactor(TURNING_FACTOR / 60.0);
            TURNING_CONFIG.closedLoop
                .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
                .pid(1,0,0)
                .outputRange(-1, 1)
                .positionWrappingEnabled(true)
                .positionWrappingInputRange(0, TURNING_FACTOR);

            FL_CONFIG
                .idleMode(IdleMode.kBrake)
                .smartCurrentLimit(50)
                .inverted(true);
            FL_CONFIG.encoder
                .positionConversionFactor(DRIVING_FACTOR) //meters
                .velocityConversionFactor(DRIVING_FACTOR / 60.0);
            FL_CONFIG.closedLoop
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                .pid(0.04, 0,0)
                .apply(new FeedForwardConfig().kV(FF_VELOCITY))
                .outputRange(-1, 1);

            FR_CONFIG
                .idleMode(IdleMode.kBrake)
                .smartCurrentLimit(50)
                .inverted(true);
            FR_CONFIG.encoder
                .positionConversionFactor(DRIVING_FACTOR) //meters
                .velocityConversionFactor(DRIVING_FACTOR / 60.0);
            FR_CONFIG.closedLoop
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                .pid(0.04, 0,0)
                .apply(new FeedForwardConfig().kV(FF_VELOCITY))
                .outputRange(-1, 1);

            BL_CONFIG
                .idleMode(IdleMode.kBrake)
                .smartCurrentLimit(50)
                .inverted(false);
            BL_CONFIG.encoder
                .positionConversionFactor(DRIVING_FACTOR) //meters
                .velocityConversionFactor(DRIVING_FACTOR / 60.0);
            BL_CONFIG.closedLoop
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                .pid(0.04, 0,0)
                .apply(new FeedForwardConfig().kV(FF_VELOCITY))
                .outputRange(-1, 1);

            BR_CONFIG
                .idleMode(IdleMode.kBrake)
                .smartCurrentLimit(50)
                .inverted(false);
            BR_CONFIG.encoder
                .positionConversionFactor(DRIVING_FACTOR) //meters
                .velocityConversionFactor(DRIVING_FACTOR / 60.0);
            BR_CONFIG.closedLoop
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                .pid(0.04, 0,0)
                .apply(new FeedForwardConfig().kV(FF_VELOCITY))
                .outputRange(-1, 1);
        }
    }

    public static final class Shooter {
        public static final SparkFlexConfig FLYWHEEL_RIGHT_CONFIG = new SparkFlexConfig();
        public static final SparkFlexConfig FLYWHEEL_LEFT_CONFIG = new SparkFlexConfig();
        public static final SparkMaxConfig FEEDER_CONFIG = new SparkMaxConfig();
        public static final SparkMaxConfig HOPPER_CONFIG = new SparkMaxConfig();
    }

    public static final class Intake {
        public static final SparkMaxConfig INTAKE_CONFIG = new SparkMaxConfig();
        public static final SparkMaxConfig ROTATE_CONFIG = new SparkMaxConfig();

        public static final double INTAKE_MOTOR_SPEED = 5.0; // temporary value
        public static final double MOTOR_HEIGHT = 5.0; // temporary value

        static {
            INTAKE_CONFIG
                .idleMode(IdleMode.kCoast)
                .inverted(true)
                .smartCurrentLimit(50);
            ROTATE_CONFIG
                .idleMode(IdleMode.kBrake)
                .smartCurrentLimit(50);
            ROTATE_CONFIG.closedLoop
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                .p(.6)
                .outputRange(-0.85, 0.85)
                .maxMotion
                .cruiseVelocity(1500)
                .maxAcceleration(750)
                .allowedProfileError(.2);
        }
    }
}
