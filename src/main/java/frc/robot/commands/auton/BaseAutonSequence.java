package frc.robot.commands.auton;

import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import frc.robot.commands.dropping.GoAndPlaceCommand;
import frc.robot.commands.grabber.RollerIntakeCommand;
import frc.robot.commands.mover.TiltedElevatorCommand;
import frc.robot.commands.swerve.FollowPathCommand;
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
            new GoAndPlaceCommand(swerveSubsystem, rollerSubsystem, tiltedElevatorSubsystem, initialPose, placePose, elevatorState)
        );
    }

    /**
     * Goes to a position and intakes a game piece.
     * @param initialPose The initial pose of the robot.
     * @param waypoints The waypoints to hit along the path.
     * @param finalPose The destination pose of the robot.
     * @return The `SequentialCommandGroup` representing running the commands in order.
     */
    protected Command goAndGrab(Pose2d initialPose, List<Pose2d> waypoints, Pose2d finalPose) {
        return goAndGrab(initialPose, waypoints, finalPose, false, false);
    }

    /**
     * Goes to a position and intakes a game piece.
     * @param initialPose The initial pose of the robot.
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
}
