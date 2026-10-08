package frc.robot.commands;

import org.wpilib.command2.Command;

import frc.robot.subsystems.GroundIntakeSubsystem;

public class IntakeToggle extends Command{
    private final GroundIntakeSubsystem intakeSub;

    public IntakeToggle(GroundIntakeSubsystem intakeSub){
        this.intakeSub = intakeSub;
        addRequirements(intakeSub);
    }

    @Override 
    
    public void initialize(){
        if(!intakeSub.isIntakeDown())
            intakeSub.extend();
        else
            intakeSub.retract();
    }

    @Override
    public void execute(){}

    @Override
    public void end(boolean interrupted){
        intakeSub.setIntake(0);
    }

    @Override
    public boolean isFinished(){
        return intakeSub.isAtSetpoint();
    }
}
