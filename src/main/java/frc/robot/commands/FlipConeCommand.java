package frc.robot.commands;

import javax.naming.InitialContext;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.TiltedElevatorSubsystem;
import frc.robot.subsystems.TiltedElevatorSubsystem.ElevatorState;
import frc.robot.subsystems.drivetrain.BaseSwerveSubsystem;

public class FlipConeCommand extends CommandBase {
    private final BaseSwerveSubsystem swerveSubsystem;
    private final TiltedElevatorSubsystem tiltedElevatorSubsystem;
    public FlipConeCommand(BaseSwerveSubsystem swerveSubsystem, TiltedElevatorSubsystem tiltedElevatorSubsystem){
        this.swerveSubsystem = swerveSubsystem;
        this.tiltedElevatorSubsystem = tiltedElevatorSubsystem;
    }
    @Override
    public void initialize() {
        tiltedElevatorSubsystem.flipCone = !tiltedElevatorSubsystem.flipCone; //"togglable" command, flipcone should init then end if flipcone is false
        addRequirements(swerveSubsystem, tiltedElevatorSubsystem);
    }

    @Override
    public void execute() {
        tiltedElevatorSubsystem.setState(ElevatorState.SLOWLYLOWER);
        swerveSubsystem.setDrivePowers(-.2);
    }

    //end if button was pressed again or elevator has reached ground(no need to continue backing up)
    @Override
    public boolean isFinished() {
        return (!tiltedElevatorSubsystem.flipCone || tiltedElevatorSubsystem.getExtension() == 0);
    }

    @Override
    public void end(boolean interrupted) {
        swerveSubsystem.setDrivePowers(0);
    }
}
