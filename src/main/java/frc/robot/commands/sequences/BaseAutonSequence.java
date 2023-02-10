package frc.robot.commands.sequences;

import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import frc.robot.commands.grabber.AidenIntakeCommand;
import frc.robot.commands.grabber.AidenPlaceCommand;
import frc.robot.commands.mover.JulianLevelCommand;
import frc.robot.commands.swerve.FollowPathCommand;
import frc.robot.positions.PiecePosition;
import frc.robot.positions.PlacePosition;
import frc.robot.subsystems.RollerSubsystem;
import frc.robot.subsystems.TiltedElevatorSubsystem;
import frc.robot.subsystems.TiltedElevatorSubsystem.ElevatorState;
import frc.robot.subsystems.drivetrain.BaseSwerveSubsystem;

public abstract class BaseAutonSequence extends SequentialCommandGroup {
    protected final BaseSwerveSubsystem swerveSubsystem;
    protected final RollerSubsystem rollerSubsystem;
    protected final TiltedElevatorSubsystem tiltedElevatorSubsystem;

    public BaseAutonSequence(
        BaseSwerveSubsystem swerveSubsystem, RollerSubsystem rollerSubsystem, TiltedElevatorSubsystem tiltedElevatorSubsystem,
        Pose2d initialPose
    ) {
        this.swerveSubsystem = swerveSubsystem;
        this.rollerSubsystem = rollerSubsystem;
        this.tiltedElevatorSubsystem = tiltedElevatorSubsystem;

        addRequirements(swerveSubsystem, rollerSubsystem, tiltedElevatorSubsystem);

        addCommands(new InstantCommand(() -> swerveSubsystem.resetPose(initialPose), swerveSubsystem));
    }

    /**
     * Goes to a position and intakes a game piece.
     * @param intialPose The initial pose of the robot.
     * @param finalPose The destination pose of the robot.
     * @return The `SequentialCommandGroup` representing running the commands in order.
     */
    protected Command goAndGrab(Pose2d initialPose, PiecePosition finalPose) {
        return FollowPathCommand.from(swerveSubsystem, initialPose, List.of(), finalPose.getPose())
            .alongWith(new JulianLevelCommand(tiltedElevatorSubsystem, ElevatorState.GROUND))
            .alongWith(new AidenIntakeCommand(rollerSubsystem));
    }

    /**
     * Goes to a position and places the currently held game piece.
     * @param intialPose The initial pose of the robot.
     * @param finalState The destination pose and elevator state of the robot.
     * @return The `SequentialCommandGroup` representing running the commands in order.
     */
    protected Command goAndPlace(Pose2d initialPose, PlacePosition finalState) {
        return FollowPathCommand.from(swerveSubsystem, initialPose, List.of(), finalState.getPose())
            .alongWith(new JulianLevelCommand(tiltedElevatorSubsystem, finalState.getElevatorState())) // or .alongWith()?
            .andThen(new AidenPlaceCommand(rollerSubsystem));
    }
}
