package frc.robot.components;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants.Configs;
import frc.robot.Constants.Drive;

public class SwerveModuleIOSim implements SwerveModuleIO {
    private final DCMotorSim drivePhysSim;
    private final DCMotorSim turnPhysSim;
    
    private final PIDController driveSimPID;
    private final PIDController turnSimPID;
    
    private SwerveModuleState targetState = new SwerveModuleState();
    private final double chassisAngularOffset;
    
    private double driveAppliedVolts = 0.0;
    private double turnAppliedVolts = 0.0;
    private double driveEncoderOffset = 0.0;
    
    public SwerveModuleIOSim(double angularOffset) {
        // Drive motor simulates linear motion of the robot
        drivePhysSim = new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                Configs.SwerveModule.Sim.DRIVE_VOLTS_PER_VELOCITY,
                Configs.SwerveModule.Sim.DRIVE_VOLTS_PER_ACCELERATION),
            DCMotor.getNEO(Configs.SwerveModule.Sim.DRIVE_MOTOR_COUNT)
        );

        // Turn motor simulates rotation of the module steering
        turnPhysSim = new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                Configs.SwerveModule.Sim.TURN_VOLTS_PER_VELOCITY,
                Configs.SwerveModule.Sim.TURN_VOLTS_PER_ACCELERATION),
            DCMotor.getNEO(Configs.SwerveModule.Sim.TURN_MOTOR_COUNT)
        );
        
        driveSimPID = new PIDController(
            Configs.SwerveModule.Sim.DRIVE_SIM_PID_P,
            Configs.SwerveModule.Sim.DRIVE_SIM_PID_I,
            Configs.SwerveModule.Sim.DRIVE_SIM_PID_D);
        turnSimPID = new PIDController(
            Configs.SwerveModule.Sim.TURN_SIM_PID_P,
            Configs.SwerveModule.Sim.TURN_SIM_PID_I,
            Configs.SwerveModule.Sim.TURN_SIM_PID_D);
        turnSimPID.enableContinuousInput(-Math.PI, Math.PI);
        
        chassisAngularOffset = angularOffset;
    }
    
    @Override
    public void updateInputs(SwerveModuleIOInputsAutoLogged inputs) {
        drivePhysSim.update(Configs.SwerveModule.Sim.SIM_TICK_TIME);
        turnPhysSim.update(Configs.SwerveModule.Sim.SIM_TICK_TIME);

        // Convert drive motor angular position/velocity to linear position/velocity
        // Motor rotations -> wheel rotations -> linear distance
        double motorRotations = (drivePhysSim.getAngularPositionRad() - driveEncoderOffset) / (2 * Math.PI);
        inputs.drivePositionMeters = motorRotations * Drive.ModuleConstants.WHEEL_CIRCUMFERENCE
            / Configs.SwerveModule.Sim.DRIVE_GEAR_RATIO;

        double motorRadPerSec = drivePhysSim.getAngularVelocityRadPerSec();
        inputs.driveVelocityMetersPerSec = motorRadPerSec * Drive.ModuleConstants.WHEEL_CIRCUMFERENCE
            / (2 * Math.PI * Configs.SwerveModule.Sim.DRIVE_GEAR_RATIO);

        inputs.driveAppliedVolts = driveAppliedVolts;
        inputs.driveCurrentAmps = drivePhysSim.getCurrentDrawAmps();

        // Convert turn motor position/velocity through gear ratio
        // Motor angle -> module angle
        inputs.turnPositionRad = (turnPhysSim.getAngularPositionRad() / Configs.SwerveModule.Sim.TURN_GEAR_RATIO)
            - chassisAngularOffset;
        inputs.turnVelocityRadPerSec = turnPhysSim.getAngularVelocityRadPerSec()
            / Configs.SwerveModule.Sim.TURN_GEAR_RATIO;

        inputs.turnAppliedVolts = turnAppliedVolts;
        inputs.turnCurrentAmps = turnPhysSim.getCurrentDrawAmps();
    }
    
    @Override
    public void setDesiredState(SwerveModuleState desiredState) {
        // Apply chassis angular offset to desired state
        SwerveModuleState correctedState = new SwerveModuleState();
        correctedState.speedMetersPerSecond = desiredState.speedMetersPerSecond;
        correctedState.angle = desiredState.angle.plus(Rotation2d.fromRadians(chassisAngularOffset));

        // Get current module angle (convert motor angle through gear ratio)
        double currentModuleAngleRad = turnPhysSim.getAngularPositionRad() / Configs.SwerveModule.Sim.TURN_GEAR_RATIO;
        Rotation2d currentAngle = new Rotation2d(currentModuleAngleRad);

        // Optimize to avoid spinning more than 90 degrees
        correctedState.optimize(currentAngle);
        targetState = correctedState;

        // Convert linear velocity to motor angular velocity for drive PID
        // m/s -> wheel rad/s -> motor rad/s
        double targetDriveMotorRadPerSec = targetState.speedMetersPerSecond
            * (2 * Math.PI * Configs.SwerveModule.Sim.DRIVE_GEAR_RATIO)
            / Drive.ModuleConstants.WHEEL_CIRCUMFERENCE;

        // Simulate closed-loop control with matching units
        // Drive PID: motor rad/s vs motor rad/s
        driveAppliedVolts = driveSimPID.calculate(
            drivePhysSim.getAngularVelocityRadPerSec(),
            targetDriveMotorRadPerSec
        );

        // Turn PID: module rad vs module rad (PID gains match hardware which works at module level)
        turnAppliedVolts = turnSimPID.calculate(
            currentModuleAngleRad,
            targetState.angle.getRadians()
        );

        // Clamp to battery voltage limits
        driveAppliedVolts = MathUtil.clamp(driveAppliedVolts, -12.0, 12.0);
        turnAppliedVolts = MathUtil.clamp(turnAppliedVolts, -12.0, 12.0);

        drivePhysSim.setInputVoltage(driveAppliedVolts);
        turnPhysSim.setInputVoltage(turnAppliedVolts);
    }
    
    @Override
    public void resetDriveEncoder() {
        driveEncoderOffset = drivePhysSim.getAngularPositionRad();
    }  
}
