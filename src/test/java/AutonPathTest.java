import org.junit.jupiter.api.Test;

import frc.robot.commands.sequences.BlueBalanceAuton;
import frc.robot.commands.sequences.BlueBottomAuton;
import frc.robot.commands.sequences.BlueTopAuton;
import frc.robot.commands.sequences.RedBalanceAuton;
import frc.robot.commands.sequences.RedBottomAuton;
import frc.robot.commands.sequences.RedTopAuton;
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
        new RedTopAuton(swerveSubsystem, rollerSubsystem, tiltedElevatorSubsystem);
        new RedBalanceAuton(swerveSubsystem, rollerSubsystem, tiltedElevatorSubsystem);
        new RedBottomAuton(swerveSubsystem, rollerSubsystem, tiltedElevatorSubsystem);
    }

    /**
     * Ensures that all blue auton paths compile.
     */
    @Test
    public void compileBluePaths() {
        new BlueTopAuton(swerveSubsystem, rollerSubsystem, tiltedElevatorSubsystem);
        new BlueBalanceAuton(swerveSubsystem, rollerSubsystem, tiltedElevatorSubsystem);
        new BlueBottomAuton(swerveSubsystem, rollerSubsystem, tiltedElevatorSubsystem);
    }
}
