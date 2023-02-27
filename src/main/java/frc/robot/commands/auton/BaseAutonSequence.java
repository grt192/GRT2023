package frc.robot.commands.auton;

import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

import frc.robot.commands.dropping.DropperChooserCommand;
import frc.robot.commands.grabber.RollerIntakeCommand;
import frc.robot.commands.mover.TiltedElevatorCommand;
import frc.robot.commands.swerve.FollowPathCommand;
import frc.robot.commands.swerve.SwerveIdleCommand;
import frc.robot.positions.PlacePosition.PlaceState;
import frc.robot.subsystems.RollerSubsystem;
import frc.robot.subsystems.drivetrain.BaseSwerveSubsystem;
import frc.robot.subsystems.tiltedelevator.ElevatorState;
import frc.robot.subsystems.tiltedelevator.TiltedElevatorSubsystem;

public abstract class BaseAutonSequence extends SequentialCommandGroup {
    protected final BaseSwerveSubsystem swerveSubsystem;
    protected final RollerSubsystem rollerSubsystem;
    protected final TiltedElevatorSubsystem tiltedElevatorSubsystem;

    public BaseAutonSequence(
        BaseSwerveSubsystem swerveSubsystem, RollerSubsystem rollerSubsystem, TiltedElevatorSubsystem tiltedElevatorSubsystem,
        Pose2d initialState
    ) {
        this.swerveSubsystem = swerveSubsystem;
        this.rollerSubsystem = rollerSubsystem;
        this.tiltedElevatorSubsystem = tiltedElevatorSubsystem;

        addRequirements(swerveSubsystem, rollerSubsystem, tiltedElevatorSubsystem);
        addCommands(
            // Reset field position
            new InstantCommand(() -> swerveSubsystem.resetPose(initialState), swerveSubsystem)
        );
    }

    /**
     * Goes to a position and intakes a game piece.
     * @param intialPose The initial pose of the robot.
     * @param waypoints The waypoints to hit along the path.
     * @param finalPose The destination pose of the robot.
     * @return The `SequentialCommandGroup` representing running the commands in order.
     */
    protected Command goAndGrab(Pose2d initialPose, List<Pose2d> waypoints, Pose2d finalPose) {
        return goAndGrab(initialPose, waypoints, finalPose, false, false);
    }

    /**
     * Goes to a position and intakes a game piece.
     * @param intialPose The initial pose of the robot.
     * @param waypoints The waypoints to hit along the path.
     * @param finalPose The destination pose of the robot.
     * @return The `SequentialCommandGroup` representing running the commands in order.
     */
    protected Command goAndGrab(Pose2d initialPose, List<Pose2d> waypoints, Pose2d finalPose, boolean startsMoving, boolean endsMoving) {
        return FollowPathCommand.composedFrom(swerveSubsystem, initialPose, waypoints, finalPose, startsMoving, endsMoving)
            .alongWith(new TiltedElevatorCommand(tiltedElevatorSubsystem, ElevatorState.GROUND))
            .deadlineWith(new RollerIntakeCommand(rollerSubsystem));
            // .andThen(new SwerveIdleCommand(swerveSubsystem));
    }

    /**
     * Goes to a position and places the currently held game piece.
     * @param intialPose The initial pose of the robot.
     * @param waypoints The waypoints to hit along the path.
     * @param raisePose The pose to raise the elevator at.
     * @param finalState The destination pose and elevator state of the robot.
     * @return The `SequentialCommandGroup` representing running the commands in order.
     */
    protected SequentialCommandGroup goAndPlace(Pose2d initialPose, List<Pose2d> waypoints, Pose2d raisePose, PlaceState finalState) {
        return FollowPathCommand.composedFrom(swerveSubsystem, initialPose, waypoints, raisePose, false, true)
            .andThen(goAndPlace(raisePose, finalState, true));
    }

    /**
     * Goes to a position and places the currently held game piece.
     * @param intialPose The initial pose of the robot.
     * @param finalState The destination pose and elevator state of the robot.
     * @return The `SequentialCommandGroup` representing running the commands in order.
     */
    protected SequentialCommandGroup goAndPlace(Pose2d initialPose, PlaceState finalState) {
        return goAndPlace(initialPose, finalState, false);
    }

    /**
     * Goes to a position and places the currently held game piece.
     * @param intialPose The initial pose of the robot.
     * @param finalState The destination pose and elevator state of the robot.
     * @param startsMoving Whether the robot starts the `FollowPathCommand` in motion.
     * @return The `SequentialCommandGroup` representing running the commands in order.
     */
    protected SequentialCommandGroup goAndPlace(Pose2d initialPose, PlaceState finalState, boolean startsMoving) {
        return new TiltedElevatorCommand(tiltedElevatorSubsystem, finalState.getElevatorState())
            .alongWith(FollowPathCommand.from(swerveSubsystem, initialPose, List.of(), finalState.getPose(), startsMoving, false))
            .andThen(new SwerveIdleCommand(swerveSubsystem))
            .andThen(new WaitCommand(0.1))
            .andThen(DropperChooserCommand.getSequence(swerveSubsystem, rollerSubsystem, tiltedElevatorSubsystem, finalState.getElevatorState()));
    }
}
