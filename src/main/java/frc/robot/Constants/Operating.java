package frc.robot.Constants;

public class Operating {
    public static final class Constants {
        public static final boolean USING_GYRO     = true;
        public static final boolean USING_DRIVE    = true;
        public static final boolean USING_VISION   = false;
        public static final boolean USING_SHOOTER  = true;
        public static final boolean USING_CLIMB    = false;
        public static final boolean USING_INTAKE   = true;
        public static final boolean USING_OPERATOR = false;
        public static final boolean USING_LIMELIGHT = false;
        public static final boolean USING_AUTO = true;

        public enum SimMode { SIM, FAST, REPLAY }

        /** From the SIM_MODE environment variable (set by build.gradle); SIM when absent or unrecognized. */
        public static SimMode simMode() {
            String v = System.getenv("SIM_MODE");
            if (v == null) return SimMode.SIM;
            return switch (v.trim().toLowerCase()) {
                case "fast" -> SimMode.FAST;
                case "replay" -> SimMode.REPLAY;
                default -> SimMode.SIM;
            };
        }
    }
}

