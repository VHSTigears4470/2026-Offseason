package frc.robot.commands;

import org.wpilib.command2.Command;

import frc.robot.subsystems.GroundIntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class ReverseIntake extends Command{
    private final GroundIntakeSubsystem intakeSub;
    private final ShooterSubsystem indexer;
    private final boolean isRunning = false;

    public ReverseIntake(GroundIntakeSubsystem intakeSub, ShooterSubsystem shooterSub) {
        this.intakeSub = intakeSub;
        this.indexer = shooterSub;
    }

    @Override 
    public void initialize(){
        // run both in reverse
        intakeSub.toggleIntake();
        if (!intakeSub.isRunning()) {
            intakeSub.setIntake(0.9);
            indexer.setIndexer(7.5);
        } else {
            intakeSub.setIntake(0);
            indexer.setIndexer(0);
        }
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

