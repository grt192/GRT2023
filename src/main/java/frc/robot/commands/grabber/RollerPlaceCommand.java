package frc.robot.commands.grabber;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.subsystems.RollerSubsystem;
import frc.robot.subsystems.RollerSubsystem.HeldPiece;

/**
 * Places 1 game piece with the roller mech. This command runs the rollers in reverse
 * at a set power for 2 seconds or until 2 seconds after the limit switch is no longer pressed.
 */
public class RollerPlaceCommand extends CommandBase {
    private final RollerSubsystem rollerSubsystem;
    private final Timer runTimer;
    private final Timer endTimer;

    private final double rollPower;
    private final double rollDuration;

    public RollerPlaceCommand(RollerSubsystem rollerSubsystem) {
        this(rollerSubsystem, -0.1, 2.0);
    }

    public RollerPlaceCommand(RollerSubsystem rollerSubsystem, double rollPower, double rollDuration) {
        this.rollerSubsystem = rollerSubsystem;
        this.runTimer = new Timer();
        this.endTimer = new Timer();

        this.rollPower = rollPower;
        this.rollDuration = rollDuration;

        addRequirements(rollerSubsystem);
    }

    @Override
    public void initialize() {
        System.out.println("Roller to place");

        rollerSubsystem.openMotor();

        runTimer.start();
    }

    @Override
    public void execute() {
        rollerSubsystem.setRollPower(rollPower);
    }

    @Override
    public void end(boolean interrupted) {
        System.out.println("Placing done");
        
        rollerSubsystem.setRollPower(0);
    }

    @Override
    public boolean isFinished() {
        /*
        boolean done = runTimer.hasElapsed(2) || rollerSubsystem.getPiece() == HeldPiece.EMPTY;
        if (done) endTimer.start();
        return endTimer.hasElapsed(2);
        */
        return runTimer.hasElapsed(rollDuration);
    }
}
