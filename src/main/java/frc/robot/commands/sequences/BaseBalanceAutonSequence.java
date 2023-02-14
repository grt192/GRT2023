package frc.robot.commands.sequences;

import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;

import frc.robot.commands.BalancerCommand;
import frc.robot.commands.swerve.FollowPathCommand;
import frc.robot.positions.PlacePosition;
import frc.robot.subsystems.RollerSubsystem;
import frc.robot.subsystems.TiltedElevatorSubsystem;
import frc.robot.subsystems.drivetrain.BaseSwerveSubsystem;

public abstract class BaseBalanceAutonSequence extends BaseAutonSequence {
    /**
     * Balancing auton sequence.
     * @param swerveSubsystem The swerve subsystem.
     * @param rollerSubsystem The roller subsystem.
     * @param tiltedElevatorSubsystem The tilted elevator subsystem.
     * @param initialPose The initial pose of the robot.
     * @param placeState The state of the robot when placing the preloaded game piece (pose and elevator state).
     * @param outsidePose The position outside the community to go to for extra points.
     */
    public BaseBalanceAutonSequence(
        BaseSwerveSubsystem swerveSubsystem, RollerSubsystem rollerSubsystem, TiltedElevatorSubsystem tiltedElevatorSubsystem,
        Pose2d initialPose, PlacePosition placeState, Pose2d midoutsidePose, Pose2d outsideoutsidePose, Pose2d backoutsidePose
    ) {
        super(swerveSubsystem, rollerSubsystem, tiltedElevatorSubsystem, initialPose);

        addCommands(
            // Place preloaded game piece
            goAndPlace(initialPose, placeState),
            // Go out of community
            FollowPathCommand.from(swerveSubsystem, placeState.getPose(), List.of(), midoutsidePose),
            //do 180
            FollowPathCommand.from(swerveSubsystem, midoutsidePose, List.of(), outsideoutsidePose),
            FollowPathCommand.from(swerveSubsystem, outsideoutsidePose, List.of(), backoutsidePose),
            // Go and balance on charging station
            new BalancerCommand(swerveSubsystem)
        );
    }
}
