package frc.robot.Constants;

import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.FeedForwardConfig;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

public final class Configs {
    public static final class SwerveModuleConfigs {
        public static final SparkMaxConfig TURNING_CONFIG = new SparkMaxConfig();
        public static final SparkFlexConfig FL_CONFIG = new SparkFlexConfig();
        public static final SparkFlexConfig FR_CONFIG = new SparkFlexConfig();
        public static final SparkFlexConfig BL_CONFIG = new SparkFlexConfig();
        public static final SparkFlexConfig BR_CONFIG = new SparkFlexConfig();
      
        static {
            //Module constants used to calculate conversion factors and feed forward gain.
            double FF_VELOCITY = 1 / Drive.ModuleConstants.DRIVE_WHEEL_FREE_RPS;

            TURNING_CONFIG
                .idleMode(IdleMode.kBrake)
                .smartCurrentLimit(20);
            TURNING_CONFIG.absoluteEncoder
                .inverted(true);
                //.positionConversionFactor(TURNING_FACTOR)         <- deprecated, set through hardware manager
                //.velocityConversionFactor(TURNING_FACTOR / 60.0); <- deprecated, set through hardware manager
            TURNING_CONFIG.closedLoop
                .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
                .pid(1,0,0)
                .outputRange(-1, 1)
                .positionWrappingEnabled(true);
                //.positionWrappingInputRange(0, TURNING_FACTOR); <- deprecated, set through hardware manager

            FL_CONFIG
                .idleMode(IdleMode.kBrake)
                .smartCurrentLimit(50)
                .inverted(true);
            //FL_CONFIG.encoder
                //.positionConversionFactor(DRIVING_FACTOR) //meters <- deprecated, set through hardware manager
                //.velocityConversionFactor(DRIVING_FACTOR / 60.0);  <- deprecated, set through hardware manager
            FL_CONFIG.closedLoop
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                .pid(0.04, 0,0)
                .apply(new FeedForwardConfig().kV(FF_VELOCITY))
                .outputRange(-1, 1);

            FR_CONFIG
                .idleMode(IdleMode.kBrake)
                .smartCurrentLimit(50)
                .inverted(false);
            //FR_CONFIG.encoder.
                //.positionConversionFactor(DRIVING_FACTOR) //meters <- deprecated, set through hardware manager
                //.velocityConversionFactor(DRIVING_FACTOR / 60.0);  <- deprecated, set through hardware manager
            FR_CONFIG.closedLoop
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                .pid(0.04, 0,0)
                .apply(new FeedForwardConfig().kV(FF_VELOCITY))
                .outputRange(-1, 1);

            BL_CONFIG
                .idleMode(IdleMode.kBrake)
                .smartCurrentLimit(50)
                .inverted(true);
            //BL_CONFIG.encoder
                //.positionConversionFactor(DRIVING_FACTOR) //meters <- deprecated, set through hardware manager
                //.velocityConversionFactor(DRIVING_FACTOR / 60.0);  <- deprecated, set through hardware manager
            BL_CONFIG.closedLoop
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                .pid(0.04, 0,0)
                .apply(new FeedForwardConfig().kV(FF_VELOCITY))
                .outputRange(-1, 1);

            BR_CONFIG
                .idleMode(IdleMode.kBrake)
                .smartCurrentLimit(50)
                .inverted(false);
            //BR_CONFIG.encoder
                //.positionConversionFactor(DRIVING_FACTOR) //meters <- deprecated, set through hardware manager
                //.velocityConversionFactor(DRIVING_FACTOR / 60.0);  <- deprecated, set through hardware manager
            BR_CONFIG.closedLoop
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                .pid(0.04, 0,0)
                .apply(new FeedForwardConfig().kV(FF_VELOCITY))
                .outputRange(-1, 1);
        }
    }

    public static final class ShooterConfigs {
        public static final SparkFlexConfig FLYWHEEL_CONFIG = new SparkFlexConfig();
        public static final SparkMaxConfig HOOD_CONFIG = new SparkMaxConfig();
        public static final SparkMaxConfig FEEDER_CONFIG = new SparkMaxConfig();
        public static final SparkMaxConfig HOPPER_CONFIG = new SparkMaxConfig();

        static {
            //Retune closed loop controller
            FLYWHEEL_CONFIG
                .idleMode(IdleMode.kCoast)
                .smartCurrentLimit(50)
                .voltageCompensation(11)
                .inverted(false); 
            FLYWHEEL_CONFIG.closedLoop
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                .apply(new FeedForwardConfig().kV(.000176)) // ~1/MAX_RPM  .00020352
                .p(0.0002) //.00078
                .d(0.25)//d can't be neg
                .outputRange(-1, 1);

            //Retune closed loop controller
            HOOD_CONFIG
                .idleMode(IdleMode.kBrake)
                .smartCurrentLimit(50);
            HOOD_CONFIG.closedLoop
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                .p(.6)
                .outputRange(-0.85, 0.85)
                .maxMotion
                .cruiseVelocity(1500)
                .maxAcceleration(750)
                .allowedProfileError(.2);

            FEEDER_CONFIG
                .idleMode(IdleMode.kBrake)
                .inverted(false) //change?
                .smartCurrentLimit(50);
            HOPPER_CONFIG
                .idleMode(IdleMode.kCoast)
                .inverted(false) //change?
                .smartCurrentLimit(50);
        }

    }
}
