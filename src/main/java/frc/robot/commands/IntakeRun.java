package frc.robot.commands;

import org.wpilib.command2.Command;

import frc.robot.subsystems.GroundIntakeSubsystem;

public class IntakeRun extends Command{
    private final GroundIntakeSubsystem intakeSub;

    public IntakeRun(GroundIntakeSubsystem intakeSub){
        this.intakeSub = intakeSub;
    }

    @Override 
    public void initialize(){
        if(!intakeSub.isRunning())
            intakeSub.setIntake(-.9);
        else
            intakeSub.setIntake(0);
    }

    @Override
    public void execute(){}

    @Override
    public void end(boolean interrupted){}

    @Override
    public boolean isFinished() {
        return true;
    }
}

