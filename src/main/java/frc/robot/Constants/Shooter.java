package frc.robot.Constants;

import org.wpilib.math.interpolation.InterpolatingDoubleTreeMap;

public final class Shooter {
    public static final class Constants {
        public static final InterpolatingDoubleTreeMap HOOD_ANGLE_TABLE = new InterpolatingDoubleTreeMap();
        public static final double HUB_RPM = -2700; // -2000;
        public static final double HUB_POSITION = 0;
        public static final double SHOOTER_THRESHOLD = -12;
        static {
            HOOD_ANGLE_TABLE.put(null, null);
        }
    }
}
