package frc.robot.commands.grabber;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.subsystems.RollerSubsystem;

/**
 * [NOT OUTDATED ANYMORE AYYYY] Places 1 game piece with the roller mech. This command runs the rollers in reverse
 * at a set power for 2 seconds.
 */
public class AidenPlaceCommand extends CommandBase {
    private final RollerSubsystem rollerSubsystem;
    private final Timer timer;
    private boolean done;

    public AidenPlaceCommand(RollerSubsystem rollerSubsystem){
        this.rollerSubsystem = rollerSubsystem;
        this.timer = new Timer();

        addRequirements(rollerSubsystem);
    }

    @Override
    public void initialize() {
        System.out.println("Roller to place");
    } 

    @Override
    public void execute() {
        //open motor to drop
        rollerSubsystem.openMotor();
        //if limit switch is still pressed (piece is still in)
        if (!rollerSubsystem.isLimit()){
            timer.start();
            rollerSubsystem.setRollPower(-0.1);
        }
        else{
            done = true;
        }
    }

    @Override
    public void end(boolean interrupted) {
        System.out.println("Placing done");
        rollerSubsystem.setRollPower(0);
    }

    @Override
    public boolean isFinished() {
        return (timer.hasElapsed(2) || done);
    }
}
