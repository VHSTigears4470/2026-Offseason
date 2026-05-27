package frc.robot.components;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.spark.SparkBase.ControlType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class PIDMotorIOSim implements PIDMotorIO {
    private final DCMotorSim motorSim;
    private final PIDController pid;
    private final PIDMotorIOInputsAutoLogged inputs = new PIDMotorIOInputsAutoLogged();

    private double kV = 0.0;
    private double setpoint = 0.0;
    private double feedforward = 0.0;
    private ControlType controlType = ControlType.kPosition;
    private boolean velocityMode = false;

    public PIDMotorIOSim(DCMotorSim motorSim, PIDController pid, double kV) {
        this.motorSim = motorSim;
        this.pid = pid;
        this.kV = kV;
    }

    @Override
    public void resetEncoder() {
        motorSim.setState(0, 0);
    }

    @Override
    public void setSetpoint(double setpoint, ControlType controlType, double FF) {
        this.setpoint = setpoint;
        this.controlType = controlType;
        this.feedforward = FF;
        this.velocityMode = false;
    }

    @Override
    public void setVelocity(double RPM, double FF) {
        this.setpoint = RPM;
        this.feedforward = FF;
        this.velocityMode = true;
    }

    @Override
    public void setVoltage(double voltage) {
        motorSim.setInputVoltage(voltage);
    }

    @Override
    public void set(double speed) {
        motorSim.setInputVoltage(speed * 12.0);
    }

    @Override
    public void stopMotors() {
        motorSim.setInputVoltage(0.0);
    }

    @Override
    public void updateInputs(PIDMotorIOInputsAutoLogged inputs) {
        inputs.RPM = motorSim.getAngularVelocityRPM();
        inputs.rotation = motorSim.getAngularPositionRotations(); // REVIEW: is this the right unit for position?
    }

    public void periodic() {
        // Simulate PID control
        double measured = velocityMode ? motorSim.getAngularVelocityRPM() : motorSim.getAngularPositionRotations();
        double feedforward = kV * setpoint;
        double ff = velocityMode ? kV * setpoint : feedforward; 
        double output = pid.calculate(measured, setpoint) + ff;
        output = Math.max(-12.0, Math.min(12.0, output));
        motorSim.setInputVoltage(output);

        motorSim.update(0.02);

        updateInputs(inputs);
        Logger.recordOutput("PIDMotor/Output", output);
        Logger.recordOutput("PIDMotor/Measured", measured);
    }
}