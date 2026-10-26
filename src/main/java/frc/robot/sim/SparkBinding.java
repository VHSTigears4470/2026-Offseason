package frc.robot.sim;

import org.wpilib.math.system.DCMotor;

import com.revrobotics.sim.SparkAbsoluteEncoderSim;
import com.revrobotics.sim.SparkFlexSim;
import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.sim.SparkRelativeEncoderSim;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkSim;

/**
 * One Spark driving one simulated degree of freedom. REV's SparkSim runs the firmware closed loop; this class feeds
 * it velocities and writes back true sensor positions, keeping offsets from encoder resets done by robot code.
 */
final class SparkBinding {
    enum Feedback { RELATIVE, ABSOLUTE }

    private final SparkSim sim;
    private final SparkRelativeEncoderSim relative;
    private final SparkAbsoluteEncoderSim absolute;
    private final Feedback feedback;
    private final boolean coastIdle;
    private double offset;
    private double lastWritten;
    private boolean outputsAllowed = true;

    SparkBinding(SparkBase spark, DCMotor motor, Feedback feedback, boolean coastIdle) {
        this.sim = spark instanceof SparkFlex flex ? new SparkFlexSim(flex, motor) : new SparkMaxSim((SparkMax) spark, motor);
        this.relative = sim.getRelativeEncoderSim();
        this.absolute = sim.getAbsoluteEncoderSim();
        this.feedback = feedback;
        this.coastIdle = coastIdle;
    }

    /** Once per robot loop, before any iterate: pick up encoder resets done by robot code. */
    void detectRobotReset() {
        double read = relative.getPosition();
        if (Math.abs(read - lastWritten) > 1e-9) {
            offset += read - lastWritten;
            lastWritten = read;
        }
    }

    /** Runs REV's controller for this substep and returns motor volts. */
    double iterate(double feedbackRpm, double vbus, double dt) {
        sim.iterate(feedbackRpm, vbus, dt);
        return sim.getAppliedOutput() * vbus;
    }

    double setpoint() {
        return sim.getSetpoint();
    }

    /** Position the closed loop compares against (absolute-feedback turn motors). */
    void setControllerPosition(double position) {
        sim.setPosition(position);
    }

    /** Writes the true relative encoder state plus the reset offset. */
    void writeRelative(double motorRotations, double motorRpm) {
        double value = motorRotations + offset;
        relative.setPosition(value);
        relative.setVelocity(motorRpm);
        if (feedback == Feedback.RELATIVE) {
            sim.setPosition(value);
        }
        lastWritten = value;
    }

    /** Writes the absolute encoder, rotations in [0, 1). */
    void writeAbsolute(double rotations, double rpm) {
        absolute.setPosition(rotations);
        absolute.setVelocity(rpm);
    }

    /** false forces zero output, as FMS would between AUTO and TELEOP and after the match. */
    void setOutputsAllowed(boolean allowed) {
        if (allowed == outputsAllowed) {
            return;
        }
        outputsAllowed = allowed;
        if (allowed) {
            sim.useDriverStationEnable();
        } else {
            sim.disable();
        }
    }

    boolean coast() {
        return coastIdle;
    }
}
