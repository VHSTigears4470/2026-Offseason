package frc.robot.Constants;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

public final class Drive {
    public static final class Constants {
        public static final double MAX_METERS_PER_SECOND = 4.46;
        public static final double MAX_ANGULAR_SPEED = 2 * Math.PI; //radians per second

        public static final double WHEEL_BASE = Units.inchesToMeters(27.5);  //change?
        public static final double TRACK_WIDTH = Units.inchesToMeters(27.5); //change?

        public static final SwerveDriveKinematics DRIVE_KINEMATICS = new SwerveDriveKinematics(
            new Translation2d(WHEEL_BASE / 2, TRACK_WIDTH / 2),    //Front Left
            new Translation2d(WHEEL_BASE / 2, -TRACK_WIDTH / 2),   //Front Right
            new Translation2d(-WHEEL_BASE / 2, TRACK_WIDTH / 2),   //Back Left
            new Translation2d(-WHEEL_BASE / 2, -TRACK_WIDTH / 2)); //Back Right

        public static final double FL_ANGULAR_OFFSET = -Math.PI/2;
        public static final double BL_ANGULAR_OFFSET =  Math.PI;
        public static final double FR_ANGULAR_OFFSET = Math.PI;
        public static final double BR_ANGULAR_OFFSET = Math.PI/2;

        public static final boolean FL_INVERTED = false; 
        public static final boolean FR_INVERTED = false; 
        public static final boolean BL_INVERTED = false; 
        public static final boolean BR_INVERTED = false; 

        public enum MotorLocation {
            FRONT_LEFT("FrontLeft"),
            FRONT_RIGHT("FrontRight"),
            BACK_LEFT("BackLeft"),
            BACK_RIGHT("BackRight");

            private final String description;

            // Enum constructors must be private or package-private
            MotorLocation(String value) {
                description = value;
            }

            public String getFieldDescription() {
                return description;
            }
        }

        public static final PIDController ROTATION_CONTROLLER = getRotationController();
        private static final PIDController getRotationController() {
            PIDController controller = new PIDController(2.5, 0.0, 0.0); 
            controller.enableContinuousInput(-Math.PI, Math.PI);
            return controller;
        }

        public static final String CONFIGS = null;
    }

    public static final class ModuleConstants {
        //MAXSwerve modules are configured with one of three pinion gears;
        //12T, 13T, or 14T. More teeth leads to a faster chassis.
        public static final int DRIVE_MOTOR_PINION_TEETH = 13; //16
        public static final double WHEEL_DIAMETER = Units.inchesToMeters(3);
        public static final double WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER * Math.PI;
        public static final double DRIVE_MOTOR_REDUCTION = (45.0 * 22.0) / (DRIVE_MOTOR_PINION_TEETH * 15); 
                                                          //(bevel * spur) /(pinion * bevel_secondary)   
        public static final double DRIVE_MOTOR_FREE_RPS = 5676.0 / 60.0;
        public static final double DRIVE_WHEEL_FREE_RPS = (DRIVE_MOTOR_FREE_RPS * WHEEL_CIRCUMFERENCE) / DRIVE_MOTOR_REDUCTION;
    }
}
