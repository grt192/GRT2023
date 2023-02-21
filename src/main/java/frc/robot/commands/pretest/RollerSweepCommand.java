package frc.robot.commands.pretest;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.RollerSubsystem;

public class RollerSweepCommand extends CommandBase {
    private final RollerSubsystem rollerSubsystem;

    private final Timer rollTimer;
    private final double sweepDuration;

    public RollerSweepCommand(RollerSubsystem rollerSubsystem, double sweepDuration) {
        this.rollerSubsystem = rollerSubsystem;

        this.rollTimer = new Timer();
        this.sweepDuration = sweepDuration;

        addRequirements(rollerSubsystem);
    }

    @Override
    public void initialize() {
        rollTimer.start();
    }

    @Override
    public void execute() {
        // Scale down timer value [0.0, ROLL_DURATION_SECONDS] to [0.0, 2.0], subtract 1
        // to sweep from [-1.0, 1.0].
        double power = rollTimer.get() / (sweepDuration / 2.0) - 1.0;
        rollerSubsystem.setRollPower(power);
    }

    @Override
    public boolean isFinished() {
        return rollTimer.hasElapsed(sweepDuration);
    }
}
