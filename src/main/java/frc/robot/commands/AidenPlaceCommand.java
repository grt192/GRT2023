package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.RollerSubsystem;

public class AidenPlaceCommand extends CommandBase {
    private final RollerSubsystem rollerSubsystem;
    private Timer timer;
    private double power;

    public AidenPlaceCommand(RollerSubsystem rollerSubsystem){
        timer = new Timer();
        power = 0.0;
        this.rollerSubsystem = rollerSubsystem;
    }

    @Override
    public void initialize() {
        System.out.println("Roller to place");
        timer.start();
    } 
    
    @Override
    public void execute() {
        if (timer.advanceIfElapsed(2)){
            power = 0;
        }
        else{
            power = -.2;
        }
        rollerSubsystem.setRollPower(power);
    }

    @Override
    public void end(boolean interrupted) {
        System.out.println("Placing done");
        timer.reset();
    }
    
    @Override
    public boolean isFinished() {
        return (power == 0);
    }
}
