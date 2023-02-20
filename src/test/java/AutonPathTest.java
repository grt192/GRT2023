import org.junit.jupiter.api.Test;

import frc.robot.commands.sequences.BalanceAutonSequence;
import frc.robot.commands.sequences.BottomAutonSequence;
import frc.robot.commands.sequences.TopAutonSequence;
import frc.robot.commands.sequences.test.BoxAutonSequence;
import frc.robot.commands.sequences.test.GRTAutonSequence;
import frc.robot.commands.sequences.test.HighRotationLinePath;
import frc.robot.commands.sequences.test.RotatingSCurveAutonSequence;
import frc.robot.commands.sequences.test.StraightLinePath;
import frc.robot.subsystems.RollerSubsystem;
import frc.robot.subsystems.TiltedElevatorSubsystem;
import frc.robot.subsystems.drivetrain.SwerveSubsystem;

public class AutonPathTest {
    private static final SwerveSubsystem swerveSubsystem = new SwerveSubsystem(null);
    private static final RollerSubsystem rollerSubsystem = new RollerSubsystem();
    private static final TiltedElevatorSubsystem tiltedElevatorSubsystem = new TiltedElevatorSubsystem();

    /**
     * Ensures that all test auton paths compile.
     */
    @Test
    public void compileTestPaths() {
        new StraightLinePath(swerveSubsystem);
        new HighRotationLinePath(swerveSubsystem);
        new RotatingSCurveAutonSequence(swerveSubsystem);
        new BoxAutonSequence(swerveSubsystem);
        new GRTAutonSequence(swerveSubsystem);
    }

    /**
     * Ensures that all red auton paths compile.
     */
    @Test
    public void compileRedPaths() {
        new TopAutonSequence(swerveSubsystem, rollerSubsystem, tiltedElevatorSubsystem, true);
        new BalanceAutonSequence(swerveSubsystem, rollerSubsystem, tiltedElevatorSubsystem, true);
        new BottomAutonSequence(swerveSubsystem, rollerSubsystem, tiltedElevatorSubsystem, true);
    }

    /**
     * Ensures that all blue auton paths compile.
     */
    @Test
    public void compileBluePaths() {
        new TopAutonSequence(swerveSubsystem, rollerSubsystem, tiltedElevatorSubsystem, false);
        new BalanceAutonSequence(swerveSubsystem, rollerSubsystem, tiltedElevatorSubsystem, false);
        new BottomAutonSequence(swerveSubsystem, rollerSubsystem, tiltedElevatorSubsystem, false);
    }
}
