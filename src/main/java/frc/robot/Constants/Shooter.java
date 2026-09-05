package frc.robot.Constants;

import org.wpilib.math.interpolation.InterpolatingDoubleTreeMap;

public final class Shooter {
    public static final class Constants {
        public static final InterpolatingDoubleTreeMap HOOD_ANGLE_TABLE = new InterpolatingDoubleTreeMap();
        public static final double SHOOTER_RPM = 4000;
        public static final double MANUAL_SHOT_POSITION = 0;
        static {
            HOOD_ANGLE_TABLE.put(null, null);
        }
    }
}
