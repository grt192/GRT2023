package frc.robot.commands.grabber;

import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.subsystems.RollerSubsystem;

/**
 * Intakes 1 game piece with the roller mech. This command runs the rollers at a set power
 * until the limit switch is triggered.
 */
public class AidenIntakeCommand extends CommandBase {
    private final RollerSubsystem rollerSubsystem;

    public AidenIntakeCommand(RollerSubsystem rollerSubsystem){
        this.rollerSubsystem = rollerSubsystem;
        addRequirements(rollerSubsystem);
    }

    @Override
    public void initialize() {
        System.out.println("Roller to intake");
    } 

    @Override
    public void execute() {
        rollerSubsystem.setRollPower(0.3);
    }

    @Override
    public void end(boolean interrupted) {
        System.out.println("Intake done");
        rollerSubsystem.setRollPower(0);
    }

    @Override
    public boolean isFinished() {
        return rollerSubsystem.hasPiece();
    }
}
