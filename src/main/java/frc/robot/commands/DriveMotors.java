package frc.robot.commands;

import org.wpilib.command2.Command;

import frc.robot.subsystems.TestMotorsSubsystem;

public class DriveMotors extends Command {
    
    private final TestMotorsSubsystem motorsSub;
    private final double speed;

    public DriveMotors(TestMotorsSubsystem motorsSub, double speed) {
        this.motorsSub = motorsSub;
        this.speed = speed;
        addRequirements(motorsSub);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        motorsSub.setSpeed(speed);
    }

    @Override
    public void end(boolean interrupted) { 
        motorsSub.stopMotor();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}