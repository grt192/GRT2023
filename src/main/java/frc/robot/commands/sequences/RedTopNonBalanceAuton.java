package frc.robot.commands.sequences;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;

import frc.robot.RobotContainer;
import frc.robot.subsystems.MoverSubsystem.MoverPosition;

/**
 * The top and bottom auton sequences, where the robot deposits its preloaded game piece and picks up
 * and places another.
 */
public class RedTopNonBalanceAuton extends BaseAutonSequence {
    private static Pose2d INITIAL_POSITION = new Pose2d();
    private static Pose2d MIDPOS1 = new Pose2d();
    private static Pose2d MIDPOS2 = new Pose2d();
    private static Pose2d MIDPOS3 = new Pose2d();

    /**
     * Constructs an auton sequence from the given parameters.
     * @param robotContainer The robot container.
     * @param placePose1 The robot's pose when placing the first game piece.
     * @param pickPose The robot's pose when intaking the second game piece.
     * @param placePose2 The robot's pose when placing the second game piece.
     * @param height The height of the mover subsystem when placing the first game piece.
     * @param height2 The height of the mover subsystem when placing the second game piece.
     */
    public RedTopNonBalanceAuton(
        RobotContainer robotContainer,  
        Pose2d placePose1, 
        Pose2d pickPose, 
        Pose2d placePose2, 
        MoverPosition height, 
        MoverPosition height2
    ) {
        super(robotContainer, INITIAL_POSITION, MIDPOS1, MIDPOS2, MIDPOS3, placePose1, pickPose, placePose2, height, height2);
    }
}
