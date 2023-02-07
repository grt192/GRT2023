package frc.robot.commands.sequences;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.subsystems.RollerSubsystem;
import frc.robot.subsystems.TiltedElevatorSubsystem;
import frc.robot.subsystems.drivetrain.BaseSwerveSubsystem;

public abstract class TopAutonSequence extends BaseAutonSequence {
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
    public TopAutonSequence(
        BaseSwerveSubsystem swerveSubsystem, RollerSubsystem rollerSubsystem, TiltedElevatorSubsystem tiltedElevatorSubsystem,
        Pose2d initialPose, Pose2d midPose1, Pose2d midPose2, Pose2d midpose3, 
        PlacePosition placeState, PiecePosition grabPose, PlacePosition placeState2
    ) {
        super(swerveSubsystem, rollerSubsystem, tiltedElevatorSubsystem);

        addCommands(
            // Place preloaded game piece
            goAndPlaceTop(initialPose, midPose1, midPose2, midpose3, placeState),
            // Go and grab 2nd piece
            goAndGrabTop(placeState.getPose(), midPose1, midPose2, midpose3, grabPose), 
            // Go and place grabbed piece
            goAndPlaceTop(grabPose.getPose(), midPose1, midPose2, midpose3, placeState2)
        );
    }
}
