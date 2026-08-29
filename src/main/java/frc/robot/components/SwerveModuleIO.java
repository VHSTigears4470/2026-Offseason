package frc.robot.components;

import org.wpilib.math.kinematics.SwerveModuleVelocity;
import org.littletonrobotics.junction.AutoLog;

public interface SwerveModuleIO {
    @AutoLog
    public class SwerveModuleIOInputs {
        public double drivePositionMeters = 0.0;
        public double driveVelocityMetersPerSec = 0.0;
        public double driveAppliedVolts = 0.0;
        public double driveCurrentAmps = 0.0;

        public double turnPositionRad = 0.0;
        public double turnVelocityRadPerSec = 0.0;
        public double turnAppliedVolts = 0.0;
        public double turnCurrentAmps = 0.0;
    }

    default void updateInputs(SwerveModuleIOInputsAutoLogged inputs) {}

    default public void setDesiredVelocity(SwerveModuleVelocity velocity) {}

    default public void resetDriveEncoder() {}
}
