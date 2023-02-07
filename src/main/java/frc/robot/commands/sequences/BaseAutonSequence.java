package frc.robot.commands.sequences;

import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import frc.robot.commands.BalancerCommand;
import frc.robot.commands.grabber.AidenIntakeCommand;
import frc.robot.commands.grabber.AidenPlaceCommand;
import frc.robot.commands.mover.JulianLevelCommand;
import frc.robot.commands.swerve.FollowPathCommand;
import frc.robot.subsystems.RollerSubsystem;
import frc.robot.subsystems.TiltedElevatorSubsystem;
import frc.robot.subsystems.TiltedElevatorSubsystem.ElevatorState;
import frc.robot.subsystems.drivetrain.BaseSwerveSubsystem;

/**
 * Auton Command Non-Balancing Sequence. Robot places pre-loaded piece, exits community to grab another, then places that peice
 */
public abstract class BaseAutonSequence extends SequentialCommandGroup {
    private final BaseSwerveSubsystem swerveSubsystem;
    private final RollerSubsystem rollerSubsystem;
    private final TiltedElevatorSubsystem tiltedElevatorSubsystem;

    /**
     * Non-balancing top auton sequence.
     * @param swerveSubsystem The swerve subsystem.
     * @param rollerSubsystem The roller subsystem.
     * @param tiltedElevatorSubsystem The tilted elevator subsystem.
     * @param initialPose The initial pose of the robot.
     * @param midPose1 The first midpose of the sequence. Avoids Charging Station.
     * @param midPose2 The second midpose of the sequence. Keeps robot in same orentiation.
     * @param midpose3 The third midpose of the sequence. Turns robot 90 degrees to grab game piece from the side (top).
     * @param placeState The state of the robot when placing the first game piece (pose and elevator state).
     * @param grabPose The pose to grab the second game piece at.
     * @param placeState2 The state of the robot when placing the second game piece (pose and elevator state).
     */
    public BaseAutonSequence(
        BaseSwerveSubsystem swerveSubsystem, RollerSubsystem rollerSubsystem, TiltedElevatorSubsystem tiltedElevatorSubsystem,
        Pose2d initialPose, Pose2d midPose1, Pose2d midPose2, Pose2d midpose3, 
        PlacePosition placeState, PiecePosition grabPose, PlacePosition placeState2
    ) {
        this.swerveSubsystem = swerveSubsystem;
        this.rollerSubsystem = rollerSubsystem;
        this.tiltedElevatorSubsystem = tiltedElevatorSubsystem;

        addRequirements(swerveSubsystem, rollerSubsystem, tiltedElevatorSubsystem);

        addCommands(
            // Place preloaded game piece
            goAndPlaceTop(initialPose, midPose1, midPose2, midpose3, placeState),
            // Go and grab 2nd piece
            goAndGrabTop(placeState.getPose(), midPose1, midPose2, midpose3, grabPose), 
            // Go and place grabbed piece
            goAndPlaceTop(grabPose.getPose(), midPose1, midPose2, midpose3, placeState2)
        );
    }

    /**
     * Non-balancing bottom auton sequence.
     * @param swerveSubsystem The swerve subsystem.
     * @param rollerSubsystem The roller subsystem.
     * @param tiltedElevatorSubsystem Thetilted elevator subsystem.
     * @param initialPose The initial pose of the robot.
     * @param midPose1 The first midpose of the sequence. Avoids the charging station.
     * @param midPose2 The second midpose of the sequence. Keeps robot in same orientation to not hit stuff.
     * @param placeState The state of the robot when placing the first game piece (pose and elevator state).
     * @param grabPose The pose to grab the second game piece at.
     * @param placeState2 The state of the robot when placing the second game piece (pose and elevator state).
     */
    public BaseAutonSequence(
        BaseSwerveSubsystem swerveSubsystem, RollerSubsystem rollerSubsystem, TiltedElevatorSubsystem tiltedElevatorSubsystem,
        Pose2d initialPose, Pose2d midPose1, Pose2d midPose2, 
        PlacePosition placeState, PiecePosition grabPose, PlacePosition placeState2
    ) {
        this.swerveSubsystem = swerveSubsystem;
        this.rollerSubsystem = rollerSubsystem;
        this.tiltedElevatorSubsystem = tiltedElevatorSubsystem;

        addRequirements(swerveSubsystem, rollerSubsystem, tiltedElevatorSubsystem);

        addCommands(
            // Place preloaded game piece
            goAndPlaceBottom(initialPose, midPose1, midPose2, placeState),
            // Go and grab 2nd piece
            goAndGrabBottom(placeState.getPose(), midPose1, midPose2, grabPose),
            // Go and place grabbed piece
            goAndPlaceBottom(grabPose.getPose(), midPose1, midPose2, placeState2)
        );
    }

    /**
     * Balancing auton sequence.
     * @param swerveSubsystem The swerve subsystem.
     * @param rollerSubsystem The roller subsystem.
     * @param tiltedElevatorSubsystem The tilted elevator subsystem.
     * @param initialPose The initial pose of the robot.
     * @param placeState The state of the robot when placing the preloaded game piece (pose and elevator state).
     * @param outsidePose The position outside the community to go to for extra points.
     */
    public BaseAutonSequence(
        BaseSwerveSubsystem swerveSubsystem, RollerSubsystem rollerSubsystem, TiltedElevatorSubsystem tiltedElevatorSubsystem,
        Pose2d initialPose, PlacePosition placeState, Pose2d outsidePose
    ) {
        this.swerveSubsystem = swerveSubsystem;
        this.rollerSubsystem = rollerSubsystem;
        this.tiltedElevatorSubsystem = tiltedElevatorSubsystem;

        addRequirements(swerveSubsystem, rollerSubsystem, tiltedElevatorSubsystem);

        addCommands(
            // Place preloaded game piece
            goAndPlace(initialPose, placeState),
            // Go out of community
            new FollowPathCommand(swerveSubsystem, placeState.getPose(), List.of(), outsidePose),
            // Go and balance on charging station
            new BalancerCommand(swerveSubsystem)
        );
    }

    /**
     * Goes to a position and intakes a game piece. Not used rn tho lol
     * @param intialPose The initial pose of the robot.
     * @param finalPose The destination pose of the robot.
     * @return The `SequentialCommandGroup` representing running the commands in order.
     */
    private Command goAndGrab(Pose2d initialPose, PiecePosition finalPose) {
        return new FollowPathCommand(swerveSubsystem, initialPose, List.of(), finalPose.getPose())
            .alongWith(new JulianLevelCommand(tiltedElevatorSubsystem, ElevatorState.GROUND))
            .alongWith(new AidenIntakeCommand(rollerSubsystem));
    }

    /**
     * Goes to a position and places the currently held game piece.
     * @param intialPose The initial pose of the robot.
     * @param finalState The destination pose and elevator state of the robot.
     * @return The `SequentialCommandGroup` representing running the commands in order.
     */
    private Command goAndPlace(Pose2d initialPose, PlacePosition finalState) {
        return new FollowPathCommand(swerveSubsystem, initialPose, List.of(), finalState.getPose())
            .alongWith(new JulianLevelCommand(tiltedElevatorSubsystem, finalState.getElevatorState())) // or .alongWith()?
            .andThen(new AidenPlaceCommand(rollerSubsystem));
    }

    /**
     * Goes to a positon to grab gamepiece, avoiding the charging station and only turning 90 degrees.
     * @param initialPose The initial pose of the robot
     * @param midPose1 Middle pose 1
     * @param midPose2 Middle pose 2 (usually the pose that turns the robot 90)
     * @param midPose3 (usually the pose that turns the robot 90)
     * @param finalPose Destination position of robot
     * @return The `SequentialCommandGroup` representing running the commands in order.
     */
    private Command goAndGrabTop(Pose2d initialPose, Pose2d midPose1, Pose2d midPose2, Pose2d midPose3, PiecePosition finalPose) {
        return new FollowPathCommand(swerveSubsystem, initialPose, List.of(), midPose1)
            .andThen(new FollowPathCommand(swerveSubsystem, midPose1, List.of(), midPose2))
            .andThen(new FollowPathCommand(swerveSubsystem, midPose2, List.of(), midPose3))
            .andThen(goAndGrab(midPose3, finalPose));
    }

    /**
     * Goes to a positon to place gamepiece, avoiding the charging station and only turning 90 degrees.
     * @param initialPose The initial pose of the robot
     * @param midPose1 Middle pose 1
     * @param midPose2 Middle pose 2 
     * @param midPose3 (usually the pose that turns the robot 90)
     * @param finalState Final state of the robot when placing the game piece.
     * @return The `SequentialCommandGroup` representing running the commands in order.
     */
    private Command goAndPlaceTop(Pose2d initialPose, Pose2d midPose1, Pose2d midPose2, Pose2d midPose3, PlacePosition finalState) {
        return new FollowPathCommand(swerveSubsystem, initialPose, List.of(), midPose1)
            .andThen(new FollowPathCommand(swerveSubsystem, midPose1, List.of(), midPose2))
            .andThen(new FollowPathCommand(swerveSubsystem, midPose2, List.of(), midPose3))
            .andThen(goAndPlace(midPose3, finalState));
    }

    /**
     * Goes to a positon to place a game piece, avoiding the charging station and only turning 90 degrees.
     * @param initialPose The initial pose of the robot
     * @param midPose1 Middle pose 1
     * @param midPose2 Middle pose 2 
     * @param finalPose Destination position of robot
     * @return The `SequentialCommandGroup` representing running the commands in order.
     */
    private Command goAndGrabBottom(Pose2d initialPose, Pose2d midPose1, Pose2d midPose2, PiecePosition finalPose) {
        return new FollowPathCommand(swerveSubsystem, initialPose, List.of(), midPose1)
            .andThen(new FollowPathCommand(swerveSubsystem, midPose1, List.of(), midPose2))
            .andThen(goAndGrab(midPose2, finalPose));
    }

    /**
     * Goes to a positon to place a game piece, avoiding the charging station and only turning 90 degrees.
     * @param initialPose The initial pose of the robot
     * @param midPose1 Middle pose 1
     * @param midPose2 Middle pose 2 
     * @param finalState Final state of the robot when placing the game piece.
     * @return The `SequentialCommandGroup` representing running the commands in order.
     */
    private Command goAndPlaceBottom(Pose2d initialPose, Pose2d midPose1, Pose2d midPose2, PlacePosition finalState) {
        return new FollowPathCommand(swerveSubsystem, initialPose, List.of(), midPose1)
            .andThen(new FollowPathCommand(swerveSubsystem, midPose1, List.of(), midPose2))
            .andThen(goAndPlace(midPose2, finalState));
    }
}
