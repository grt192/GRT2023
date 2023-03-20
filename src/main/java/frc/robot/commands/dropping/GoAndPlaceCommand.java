package frc.robot.commands.dropping;

import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

import frc.robot.commands.mover.TiltedElevatorCommand;
import frc.robot.commands.swerve.FollowPathCommand;
import frc.robot.commands.swerve.SwerveIdleCommand;
import frc.robot.subsystems.RollerSubsystem;
import frc.robot.subsystems.drivetrain.BaseSwerveSubsystem;
import frc.robot.subsystems.tiltedelevator.ElevatorState;
import frc.robot.subsystems.tiltedelevator.TiltedElevatorSubsystem;

public class GoAndPlaceCommand extends SequentialCommandGroup {
    /**
     * Goes to a position and places the currently held game piece.
     * @param swerveSubsystem The swerve subsystem.
     * @param rollerSubsystem The roller subsystem.
     * @param tiltedElevatorSubsystem The tilted elevator subsystem.
     * @param initialPose The initial pose of the robot.
     * @param finalPose The destination pose of the robot.
     * @param elevatorState The destination elevator state of the robot.
     * @param startsMoving Whether the robot starts the `FollowPathCommand` in motion.
     */
    public GoAndPlaceCommand(
        BaseSwerveSubsystem swerveSubsystem, RollerSubsystem rollerSubsystem, TiltedElevatorSubsystem tiltedElevatorSubsystem,
        Pose2d initialPose, Pose2d finalPose, ElevatorState elevatorState, boolean startsMoving
    ) {
        addRequirements(swerveSubsystem, rollerSubsystem, tiltedElevatorSubsystem);

        addCommands(
            new TiltedElevatorCommand(tiltedElevatorSubsystem, elevatorState).alongWith(
                FollowPathCommand.from(swerveSubsystem, initialPose, List.of(), finalPose, startsMoving, false)
            ),
            new SwerveIdleCommand(swerveSubsystem),
            new WaitCommand(0.1),
            DropperChooserCommand.getSequence(swerveSubsystem, rollerSubsystem, tiltedElevatorSubsystem, elevatorState)
        );
    }

    /**
     * Goes to a position and places the currently held game piece.
     * @param swerveSubsystem The swerve subsystem.
     * @param rollerSubsystem The roller subsystem.
     * @param tiltedElevatorSubsystem The tilted elevator subsystem.
     * @param initialPose The initial pose of the robot.
     * @param finalPose The destination pose of the robot.
     * @param elevatorState The destination elevator state of the robot.
     */
    public GoAndPlaceCommand(
        BaseSwerveSubsystem swerveSubsystem, RollerSubsystem rollerSubsystem, TiltedElevatorSubsystem tiltedElevatorSubsystem,
        Pose2d initialPose, Pose2d finalPose, ElevatorState elevatorState
    ) {
        this(swerveSubsystem, rollerSubsystem, tiltedElevatorSubsystem, initialPose, finalPose, elevatorState, false);
    }

    /**
     * Goes to a position and places the currently held game piece.
     * @param swerveSubsystem The swerve subsystem.
     * @param rollerSubsystem The roller subsystem.
     * @param tiltedElevatorSubsystem The tilted elevator subsystem.
     * @param initialPose The initial pose of the robot.
     * @param waypoints The waypoints to hit along the path.
     * @param raisePose The pose to raise the elevator at.
     * @param finalPose The destination pose of the robot.
     * @param elevatorState The destination elevator state of the robot.
     * @return The `SequentialCommandGroup` representing running the commands in order.
     */
    public static SequentialCommandGroup composedFrom(
        BaseSwerveSubsystem swerveSubsystem, RollerSubsystem rollerSubsystem, TiltedElevatorSubsystem tiltedElevatorSubsystem,
        Pose2d initialPose, List<Pose2d> waypoints, Pose2d raisePose, Pose2d finalPose, ElevatorState elevatorState
    ) {
        return FollowPathCommand.composedFrom(swerveSubsystem, initialPose, waypoints, raisePose, false, true)
            .andThen(new GoAndPlaceCommand(swerveSubsystem, rollerSubsystem, tiltedElevatorSubsystem, raisePose, finalPose, elevatorState, true));
    }
}
