package frc.robot.commands.swerve;

import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import frc.robot.commands.grabber.RollerPlaceCommand;
import frc.robot.commands.mover.TiltedElevatorCommand;
import frc.robot.positions.PlacePosition;
import frc.robot.subsystems.RollerSubsystem;
import frc.robot.subsystems.TiltedElevatorSubsystem;
import frc.robot.subsystems.drivetrain.BaseSwerveSubsystem;

public class GoAndPlaceCommand extends SequentialCommandGroup {
    public GoAndPlaceCommand(
        BaseSwerveSubsystem swerveSubsystem, RollerSubsystem rollerSubsystem, TiltedElevatorSubsystem tiltedElevatorSubsystem,
        Pose2d initialPose, PlacePosition placePosition
    ) {
        addRequirements(swerveSubsystem, tiltedElevatorSubsystem, rollerSubsystem);
        addCommands(
            FollowPathCommand.from(
                swerveSubsystem, 
                initialPose, 
                List.of(), 
                placePosition.getPose()
            ).alongWith(
                new TiltedElevatorCommand(tiltedElevatorSubsystem, placePosition.getElevatorState())
            ),
            new RollerPlaceCommand(rollerSubsystem)
        );
    }
}
