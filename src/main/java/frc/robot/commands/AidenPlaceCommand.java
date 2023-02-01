package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.subsystems.RollerSubsystem;

/**
 * [OUTDATED] Places 1 game piece with the roller mech. This command runs the rollers in reverse
 * at a set power for 2 seconds.
 */
public class AidenPlaceCommand extends CommandBase {
    private final RollerSubsystem rollerSubsystem;
    private final Timer timer;

    public AidenPlaceCommand(RollerSubsystem rollerSubsystem){
        this.rollerSubsystem = rollerSubsystem;
        this.timer = new Timer();

        addRequirements(rollerSubsystem);
    }

    @Override
    public void initialize() {
        System.out.println("Roller to place");
        timer.start();
    } 

    @Override
    public void execute() {
        rollerSubsystem.setRollPower(-0.2);
    }

    @Override
    public void end(boolean interrupted) {
        System.out.println("Placing done");
        rollerSubsystem.setRollPower(0);
    }

    @Override
    public boolean isFinished() {
        return timer.hasElapsed(2);
    }
}
