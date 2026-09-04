package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;

import org.wpilib.command2.SubsystemBase;

import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.FeedForwardConfig;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;


public class TestMotorsSubsystem extends SubsystemBase{

    private final SparkMax max;
    private final SparkFlex flex;
    private final SparkMaxConfig maxConfig =  new SparkMaxConfig();
    private final SparkFlexConfig flexConfig = new SparkFlexConfig();

    boolean isMax;

    public TestMotorsSubsystem(int busID, int ID, boolean isMax) {

        maxConfig.idleMode(IdleMode.kBrake)
            .smartCurrentLimit(50)
            .inverted(false);
        //maxConfig.encoder;
        maxConfig.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .outputRange(-1, 1);

        flexConfig.idleMode(IdleMode.kBrake)
            .smartCurrentLimit(50)
            .inverted(false);
        //maxConfig.encoder;
        flexConfig.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .outputRange(-1, 1);

        this.isMax = isMax;
        
        if(isMax) {
            max = new SparkMax(busID, ID, MotorType.kBrushless);
            flex = null;
        } else {
            max = null;
            flex = new SparkFlex(busID, ID, MotorType.kBrushless);
        }
    }

    public void setSpeed(double speed) {
        speed = Math.signum(speed) * Math.min(Math.max(Math.abs(speed), 0), 1);
        if(isMax)
            max.setThrottle(speed);
        else 
            flex.setThrottle(speed);
    }

    public void stopMotor() {
        if(isMax)
            max.stopMotor();
        else 
            flex.stopMotor();
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run;
    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
    }

}