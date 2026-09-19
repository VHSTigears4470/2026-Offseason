package frc.robot.components;

import org.littletonrobotics.junction.AutoLog;

import com.revrobotics.spark.SparkLowLevel;

public interface PIDMotorIO {
    @AutoLog
    public class PIDMotorIOInputs {
        public double velocity = 0.0;
        //add the rest
    }

    default void updateInputs(PIDMotorIOInputsAutoLogged inputs){}

    default public void resetEncoder(){}

    default public void setSetpoint(double setpoint, SparkLowLevel.ControlType controlType, double FF){}

    default public void setVelocity(double velocity, double FF){}

    default public void set(double speed){}

    default public void setVoltage(double voltage){}

    default public void setEncoder(double setpoint){}

    default public void stopMotors(){}

    default public double getEncoder(){return 0;}

    default public double getEncoderAbs(){return 0;}

    default public double getVelocity(){return 0;}

    default public double getAppliedOutput(){return 0;}

    default public double getBusVoltage(){return 0;}

    default public double getOutputCurrent(){return 0;}
}
