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
            double FF_VELOCITY = 1 / Drive.ModuleConstants.DRIVE_WHEEL_FREE_MPS / 60;

            TURNING_CONFIG
                .idleMode(IdleMode.kBrake)
                .smartCurrentLimit(20)
                .secondaryCurrentLimit(70);
            TURNING_CONFIG.absoluteEncoder
                .inverted(true);
                //.positionConversionFactor(TURNING_FACTOR)         <- deprecated, set through hardware manager
                //.velocityConversionFactor(TURNING_FACTOR / 60.0); <- deprecated, set through hardware manager
            TURNING_CONFIG.closedLoop
                .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
                .p(3)
                .outputRange(-1, 1)
                .positionWrappingEnabled(true);
                //.positionWrappingInputRange(0, TURNING_FACTOR); <- deprecated, set through hardware manager

            FL_CONFIG
                .idleMode(IdleMode.kBrake)
                .smartCurrentLimit(40)
                .secondaryCurrentLimit(90)
                .inverted(true);
            //FL_CONFIG.encoder
                //.positionConversionFactor(DRIVING_FACTOR) //meters <- deprecated, set through hardware manager
                //.velocityConversionFactor(DRIVING_FACTOR / 60.0);  <- deprecated, set through hardware manager
            FL_CONFIG.closedLoop
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                .p(0.0001)
                .apply(new FeedForwardConfig().kV(FF_VELOCITY))
                .outputRange(-1, 1);

            FR_CONFIG
                .idleMode(IdleMode.kBrake)
                .smartCurrentLimit(40)
                .secondaryCurrentLimit(90)
                .inverted(false);
            //FR_CONFIG.encoder
                //.positionConversionFactor(DRIVING_FACTOR) //meters <- deprecated, set through hardware manager
                //.velocityConversionFactor(DRIVING_FACTOR / 60.0);  <- deprecated, set through hardware manager
            FR_CONFIG.closedLoop
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                .p(0.0001)
                .apply(new FeedForwardConfig().kV(FF_VELOCITY))
                .outputRange(-1, 1);

            BL_CONFIG
                .idleMode(IdleMode.kBrake)
                .smartCurrentLimit(40)
                .secondaryCurrentLimit(90)
                .inverted(true);
            // BL_CONFIG.encoder
                // .positionConversionFactor(DRIVING_FACTOR) //meters <- deprecated, set through hardware manager
                //.velocityConversionFactor(DRIVING_FACTOR / 60.0);  <- deprecated, set through hardware manager
            BL_CONFIG.closedLoop
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                .p(0.0001) 
                .apply(new FeedForwardConfig().kV(FF_VELOCITY))
                .outputRange(-1, 1);

            BR_CONFIG
                .idleMode(IdleMode.kBrake)
                .smartCurrentLimit(40)
                .secondaryCurrentLimit(90)
                .inverted(false);
            //BR_CONFIG.encoder
                //.positionConversionFactor(DRIVING_FACTOR) //meters <- deprecated, set through hardware manager
                //.velocityConversionFactor(DRIVING_FACTOR / 60.0);  <- deprecated, set through hardware manager
            BR_CONFIG.closedLoop
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                .p(0.0001)
                .apply(new FeedForwardConfig().kV(FF_VELOCITY))
                .outputRange(-1, 1);
        }
    }

    public static final class ShooterConfigs {
        public static final SparkFlexConfig FLYWHEEL_CONFIG = new SparkFlexConfig();
        public static final SparkMaxConfig HOOD_CONFIG = new SparkMaxConfig();
        public static final SparkMaxConfig FEEDER_CONFIG = new SparkMaxConfig();
        public static final SparkMaxConfig INDEXER_CONFIG = new SparkMaxConfig();

        static {
            //Retune closed loop controller
            FLYWHEEL_CONFIG
                .idleMode(IdleMode.kCoast)
                .smartCurrentLimit(60) // FIX: this
                .secondaryCurrentLimit(85)
                .voltageCompensation(12)
                .inverted(false); 
            FLYWHEEL_CONFIG.closedLoop
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                .apply(new FeedForwardConfig().kV(.00205)) // ~1/MAX_RPM .00215 - 2000RPM
                .p(0.002) //.0005 - 2000 RPM
                // .d(0.02)//.25 - 2000 RPM
                .outputRange(-1, 1)
                .maxMotion
                .maxAcceleration(1500)
                .cruiseVelocity(2100);
            //Retune closed loop controller
            HOOD_CONFIG
                .idleMode(IdleMode.kBrake)
                .smartCurrentLimit(40)
                .secondaryCurrentLimit(80);
            HOOD_CONFIG.closedLoop
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                .p(0.3)
                .outputRange(-0.4, 0.4);
        
            FEEDER_CONFIG
                .idleMode(IdleMode.kBrake)
                .inverted(false) //change?
                .smartCurrentLimit(25);
            INDEXER_CONFIG
                .idleMode(IdleMode.kCoast)
                .inverted(false) //change?
                .smartCurrentLimit(25);
        }
        public static final class IntakeConfigs { 
            public static final SparkMaxConfig INTAKE_CONFIG = new SparkMaxConfig(); 
            public static final SparkMaxConfig DROP_CONFIG = new SparkMaxConfig();

            static {
                INTAKE_CONFIG
                    .idleMode(IdleMode.kCoast)
                    .smartCurrentLimit(25);
                DROP_CONFIG
                    .idleMode(IdleMode.kBrake)
                    .smartCurrentLimit(40);
                DROP_CONFIG.closedLoop
                    .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                    .p(.75)
                    .outputRange(-0.85, 0.85)
                    .maxMotion
                    .cruiseVelocity(1200)
                    .maxAcceleration(600)
                    .allowedProfileError(.15);
            }
        }
    }
}
