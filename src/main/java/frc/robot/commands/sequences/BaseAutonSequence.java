package frc.robot.commands.sequences;

import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import frc.robot.RobotContainer;
import frc.robot.commands.BalancerCommand;
import frc.robot.commands.MatthewIntakeCommand;
import frc.robot.commands.MatthewPlaceCommand;
import frc.robot.commands.ShiraLevelCommand;
import frc.robot.commands.swerve.FollowPathCommand;
import frc.robot.subsystems.GripperSubsytem;
import frc.robot.subsystems.MoverSubsystem;
import frc.robot.subsystems.MoverSubsystem.MoverPosition;
import frc.robot.subsystems.drivetrain.BaseSwerveSubsystem;

/**
 * Auton Command Non-Balancing Sequence. Robot places pre-loaded piece, exits community to grab another, then places that peice
 */
public abstract class BaseAutonSequence extends SequentialCommandGroup {
    private final BaseSwerveSubsystem swerveSubsystem;
    private final GripperSubsytem gripperSubsytem;
    private final MoverSubsystem moverSubsystem;

    //positions for placing gamepieces
    //all positions from center of robot
    // ^^^ from center of robot? aren't these field coordinates?
    public enum RedPositions {
        X1(new Pose2d(
            Units.inchesToMeters(578.737),
            Units.inchesToMeters(195.55),
            Rotation2d.fromDegrees(0)
        )),
        X2(new Pose2d(
            Units.inchesToMeters(578.737),
            Units.inchesToMeters(174.123),
            Rotation2d.fromDegrees(0)
        )),
        X3(new Pose2d(
            Units.inchesToMeters(578.737),
            Units.inchesToMeters(152.123),
            Rotation2d.fromDegrees(0)
        )),
        X4(new Pose2d(
            Units.inchesToMeters(578.737),
            Units.inchesToMeters(129.75),
            Rotation2d.fromDegrees(0)
        )),
        X5(new Pose2d(
            Units.inchesToMeters(578.737),
            Units.inchesToMeters(107.801),
            Rotation2d.fromDegrees(0)
        )),
        X6(new Pose2d(
            Units.inchesToMeters(578.737),
            Units.inchesToMeters(86.149),
            Rotation2d.fromDegrees(0)
        )),
        X7(new Pose2d(
            Units.inchesToMeters(578.737),
            Units.inchesToMeters(64.818),
            Rotation2d.fromDegrees(0)
        )),
        X8(new Pose2d(
            Units.inchesToMeters(578.737),
            Units.inchesToMeters(41.761),
            Rotation2d.fromDegrees(0)
        )),
        X9(new Pose2d(
            Units.inchesToMeters(578.737),
            Units.inchesToMeters(20.016),
            Rotation2d.fromDegrees(0)
        )),

        PIECE1(new Pose2d(
            Units.inchesToMeters(0),
            Units.inchesToMeters(0),
            Rotation2d.fromDegrees(0)
        )),
        PIECE2(new Pose2d(
            Units.inchesToMeters(0),
            Units.inchesToMeters(0),
            Rotation2d.fromDegrees(0)
        )),
        PIECE3(new Pose2d(
            Units.inchesToMeters(0),
            Units.inchesToMeters(0),
            Rotation2d.fromDegrees(0)
        )),
        PIECE4(new Pose2d(
            Units.inchesToMeters(0),
            Units.inchesToMeters(0),
            Rotation2d.fromDegrees(0)
        )),

        TOP_INIT(new Pose2d(
            Units.inchesToMeters(581.072),
            Units.inchesToMeters(195.55),
            Rotation2d.fromDegrees(0)
        )),
        BALANCE_INIT(new Pose2d(
            Units.inchesToMeters(581.072),
            Units.inchesToMeters(107.638),
            Rotation2d.fromDegrees(0)
        )),
        BOTTOM_INIT(new Pose2d(
            Units.inchesToMeters(581.072),
            Units.inchesToMeters(12.873),
            Rotation2d.fromDegrees(0)
        ));

        public Pose2d position;

        private RedPositions(Pose2d position){
            this.position = position;
        }
    } 

    //positions for placing gamepieces
    public enum BluePositions {
        X1(new Pose2d(
            Units.inchesToMeters(72.013),
            Units.inchesToMeters(195.55),
            Rotation2d.fromDegrees(180)
        )),
        X2(new Pose2d(
            Units.inchesToMeters(72.013),
            Units.inchesToMeters(174.123),
            Rotation2d.fromDegrees(180)
        )),
        X3(new Pose2d(
            Units.inchesToMeters(72.013),
            Units.inchesToMeters(152.123),
            Rotation2d.fromDegrees(180)
        )),
        X4(new Pose2d(
            Units.inchesToMeters(72.013),
            Units.inchesToMeters(129.75),
            Rotation2d.fromDegrees(180)
        )),
        X5(new Pose2d(
            Units.inchesToMeters(72.013),
            Units.inchesToMeters(107.801),
            Rotation2d.fromDegrees(180)
        )),
        X6(new Pose2d(
            Units.inchesToMeters(72.013),
            Units.inchesToMeters(86.149),
            Rotation2d.fromDegrees(180)
        )),
        X7(new Pose2d(
            Units.inchesToMeters(72.013),
            Units.inchesToMeters(64.818),
            Rotation2d.fromDegrees(180)
        )),
        X8(new Pose2d(
            Units.inchesToMeters(72.013),
            Units.inchesToMeters(41.761),
            Rotation2d.fromDegrees(180)
        )),
        X9(new Pose2d(
            Units.inchesToMeters(72.013),
            Units.inchesToMeters(20.016),
            Rotation2d.fromDegrees(180)
        )),

        PIECE1(new Pose2d(
            Units.inchesToMeters(0),
            Units.inchesToMeters(0),
            Rotation2d.fromDegrees(0)
        )),
        PIECE2(new Pose2d(
            Units.inchesToMeters(0),
            Units.inchesToMeters(0),
            Rotation2d.fromDegrees(0)
        )),
        PIECE3(new Pose2d(
            Units.inchesToMeters(0),
            Units.inchesToMeters(0),
            Rotation2d.fromDegrees(0)
        )),
        PIECE4(new Pose2d(
            Units.inchesToMeters(0),
            Units.inchesToMeters(0),
            Rotation2d.fromDegrees(0)
        )),

        TOP_INIT(new Pose2d(
            Units.inchesToMeters(70.007),
            Units.inchesToMeters(195.55),
            Rotation2d.fromDegrees(180)
        )),
        BALANCE_INIT(new Pose2d(
            Units.inchesToMeters(70.007),
            Units.inchesToMeters(107.638),
            Rotation2d.fromDegrees(180)
        )),
        BOTTOM_INIT(new Pose2d(
            Units.inchesToMeters(70.007),
            Units.inchesToMeters(12.873),
            Rotation2d.fromDegrees(180)
        ));

        public Pose2d position;

        private BluePositions(Pose2d position){
            this.position = position;
        }
    }

    // non balancing auton sequence
    public BaseAutonSequence(RobotContainer robotContainer, Pose2d initialPose, Pose2d placePose, Pose2d grabPose, Pose2d placePose2, MoverPosition moverPosition, MoverPosition moverPosition2) {
        swerveSubsystem = robotContainer.getSwerveSubsystem();
        gripperSubsytem = robotContainer.getGripperSubsytem();
        moverSubsystem = robotContainer.getMoverSubsystem();
 
        addRequirements(swerveSubsystem, gripperSubsytem, moverSubsystem);

        addCommands(
            // Place preloaded game piece
            goAndPlace(initialPose, placePose, moverPosition),
            // Go and grab 2nd piece
            goAndGrab(placePose, grabPose),
            // Go and place grabbed piece
            goAndPlace(grabPose, placePose2, moverPosition2)
        );
    }

    // balancing auton sequence
    public BaseAutonSequence(RobotContainer robotContainer, Pose2d initialPose, Pose2d placePose, Pose2d outsidePose, MoverPosition moverPosition) {
        swerveSubsystem = robotContainer.getSwerveSubsystem();
        gripperSubsytem = robotContainer.getGripperSubsytem();
        moverSubsystem = robotContainer.getMoverSubsystem();

        addRequirements(swerveSubsystem, gripperSubsytem, moverSubsystem);

        addCommands(
            // Place preloaded game piece
            goAndPlace(initialPose, placePose, moverPosition),
            // Go out of community
            new FollowPathCommand(swerveSubsystem, placePose, List.of(), outsidePose),
            // Go and balance on charging station
            new BalancerCommand(swerveSubsystem)
        );
    }

    /**
     * Goes to a position and intakes a game piece.
     * @param intialPose The initial pose of the robot.
     * @param finalPose The destination pose of the robot.
     * @return The `SequentialCommandGroup` representing running the commands in order.
     */
    private Command goAndGrab(Pose2d initialPose, Pose2d finalPose) {
        return new FollowPathCommand(swerveSubsystem, initialPose, List.of(), finalPose)
            .andThen(new ShiraLevelCommand(moverSubsystem, MoverPosition.GROUND)) // or .alongWith()?
            .andThen(new MatthewIntakeCommand(gripperSubsytem));
    }

    /**
     * Goes to a position and places the currently held game piece.
     * @param intialPose The initial pose of the robot.
     * @param finalPose The destination pose of the robot.
     * @param moverPosition The target position of the mover.
     * @return The `SequentialCommandGroup` representing running the commands in order.
     */
    private Command goAndPlace(Pose2d intialPose, Pose2d finalPose, MoverPosition moverPosition) {
        return new FollowPathCommand(swerveSubsystem, intialPose, List.of(), finalPose)
            .andThen(new ShiraLevelCommand(moverSubsystem, moverPosition)) // or .alongWith()?
            .andThen(new MatthewPlaceCommand(gripperSubsytem));
    }
}
