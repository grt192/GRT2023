package frc.robot.commands.auton;

import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

import frc.robot.commands.dropping.DropperChooserCommand;
import frc.robot.commands.grabber.RollerIntakeCommand;
import frc.robot.commands.mover.TiltedElevatorCommand;
import frc.robot.commands.swerve.FollowPathCommand;
import frc.robot.commands.swerve.SwerveIdleCommand;
import frc.robot.positions.PlacePosition;
import frc.robot.subsystems.RollerSubsystem;
import frc.robot.subsystems.drivetrain.BaseSwerveSubsystem;
import frc.robot.subsystems.tiltedelevator.ElevatorState;
import frc.robot.subsystems.tiltedelevator.TiltedElevatorSubsystem;

public abstract class BaseAutonSequence extends SequentialCommandGroup {
    protected final BaseSwerveSubsystem swerveSubsystem;
    protected final RollerSubsystem rollerSubsystem;
    protected final TiltedElevatorSubsystem tiltedElevatorSubsystem;

    protected BaseAutonSequence(
        BaseSwerveSubsystem swerveSubsystem, RollerSubsystem rollerSubsystem, TiltedElevatorSubsystem tiltedElevatorSubsystem,
        PlacePosition initialPlacePosition, boolean isRed
    ) {
        this.swerveSubsystem = swerveSubsystem;
        this.rollerSubsystem = rollerSubsystem;
        this.tiltedElevatorSubsystem = tiltedElevatorSubsystem;

        addRequirements(swerveSubsystem, rollerSubsystem, tiltedElevatorSubsystem);

        Pose2d initialPose = initialPlacePosition.alignPosition.getPose(isRed);
        Pose2d placePose = initialPlacePosition.placePosition.getPose(isRed);
        ElevatorState elevatorState = initialPlacePosition.elevatorState;

        addCommands(
            // Reset field position and angle; the robot always faces towards the driver station at 180 degrees.
            new InstantCommand(() -> swerveSubsystem.resetDriverHeading(Rotation2d.fromDegrees(180)), swerveSubsystem),
            new InstantCommand(() -> swerveSubsystem.resetPose(initialPose), swerveSubsystem),

            // Place preloaded game piece
            goAndPlace(initialPose, placePose, elevatorState)
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
     * @param finalPose The destination pose of the robot.
     * @param elevatorState The destination elevator state of the robot.
     * @return The `SequentialCommandGroup` representing running the commands in order.
     */
    protected SequentialCommandGroup goAndPlace(Pose2d initialPose, List<Pose2d> waypoints, Pose2d raisePose, Pose2d finalPose, ElevatorState elevatorState) {
        return FollowPathCommand.composedFrom(swerveSubsystem, initialPose, waypoints, raisePose, false, true)
            .andThen(goAndPlace(raisePose, finalPose, elevatorState, true));
    }

    /**
     * Goes to a position and places the currently held game piece.
     * @param intialPose The initial pose of the robot.
     * @param finalPose The destination pose of the robot.
     * @param elevatorState The destination elevator state of the robot.
     * @return The `SequentialCommandGroup` representing running the commands in order.
     */
    protected SequentialCommandGroup goAndPlace(Pose2d initialPose, Pose2d finalPose, ElevatorState elevatorState) {
        return goAndPlace(initialPose, finalPose, elevatorState, false);
    }

    /**
     * Goes to a position and places the currently held game piece.
     * @param intialPose The initial pose of the robot.
     * @param finalPose The destination pose of the robot.
     * @param elevatorState The destination elevator state of the robot.
     * @param startsMoving Whether the robot starts the `FollowPathCommand` in motion.
     * @return The `SequentialCommandGroup` representing running the commands in order.
     */
    protected SequentialCommandGroup goAndPlace(Pose2d initialPose, Pose2d finalPose, ElevatorState elevatorState, boolean startsMoving) {
        return new TiltedElevatorCommand(tiltedElevatorSubsystem, elevatorState)
            .alongWith(FollowPathCommand.from(swerveSubsystem, initialPose, List.of(), finalPose, startsMoving, false))
            .andThen(new SwerveIdleCommand(swerveSubsystem))
            .andThen(new WaitCommand(0.1))
            .andThen(DropperChooserCommand.getSequence(swerveSubsystem, rollerSubsystem, tiltedElevatorSubsystem, elevatorState));
    }
}
