package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.MoverSubsystem;
import frc.robot.subsystems.MoverSubsystem.MoverPosition;

public class ShiraLevelCommand extends CommandBase{
    private final MoverSubsystem moverSubsystem;
    private final MoverPosition level;

    public ShiraLevelCommand(MoverSubsystem moverSubsystem, MoverPosition level){
        this.moverSubsystem = moverSubsystem;
        this.level = level;
    }
    
    @Override
    public void initialize() {
        System.out.println("Mover to " + level);
    }

    @Override
    public void execute() {
        moverSubsystem.setState(level);
    }

    @Override
    public void end(boolean interrupted) {
        System.out.println("Mover at " + level);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
    
}
