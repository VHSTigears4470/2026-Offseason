package frc.robot.Constants;

public class States {

    public enum RobotMode {
        IDLE,
        INTAKING_PREP,
        INTAKING,
        SHOOTING_PREP,
        SHOOTING,
        CLIMBING_PREP,
        CLIMBING
    }

    public enum GameMode {
        INITIAL,
        AUTONOMOUS,
        TRANSITION,
        SHIFT_1,
        SHIFT_2,
        SHIFT_3,
        SHIFT_4,
        ENDGAME
    }
}
