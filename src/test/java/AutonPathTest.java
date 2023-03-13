import org.junit.jupiter.api.Test;

import frc.robot.commands.auton.BalanceAndTaxiAutonSequence;
import frc.robot.commands.auton.BalanceAutonSequence;
import frc.robot.commands.auton.BottomBalanceAutonSequence;
import frc.robot.commands.auton.BottomOnePieceAutonSequence;
import frc.robot.commands.auton.BottomTwoPieceAutonSequence;
import frc.robot.commands.auton.PreloadedOnlyAutonSequence;
import frc.robot.commands.auton.TopOnePieceAutonSequence;
import frc.robot.commands.auton.TopTwoPieceAutonSequence;
import frc.robot.commands.auton.test.BoxAutonSequence;
import frc.robot.commands.auton.test.ContinuousBoxAutonSequence;
import frc.robot.commands.auton.test.GRTAutonSequence;
import frc.robot.commands.auton.test.HighRotationLinePath;
import frc.robot.commands.auton.test.RotatingSCurveAutonSequence;
import frc.robot.commands.auton.test.TenFeetStraightLinePath;
import frc.robot.commands.auton.test.TwentyFeetStraightLinePath;
import frc.robot.subsystems.RollerSubsystem;
import frc.robot.subsystems.drivetrain.SwerveSubsystem;
import frc.robot.subsystems.tiltedelevator.TiltedElevatorSubsystem;

public class AutonPathTest {
    private static final SwerveSubsystem swerveSubsystem = new SwerveSubsystem(null, null);
    private static final RollerSubsystem rollerSubsystem = new RollerSubsystem();
    private static final TiltedElevatorSubsystem tiltedElevatorSubsystem = new TiltedElevatorSubsystem();

    /**
     * Ensures that all test auton paths compile.
     */
    @Test
    public void compileTestPaths() {
        new TenFeetStraightLinePath(swerveSubsystem);
        new TwentyFeetStraightLinePath(swerveSubsystem);
        new HighRotationLinePath(swerveSubsystem);
        new RotatingSCurveAutonSequence(swerveSubsystem);
        new BoxAutonSequence(swerveSubsystem);
        new ContinuousBoxAutonSequence(swerveSubsystem);
        new GRTAutonSequence(swerveSubsystem);
    }

    /**
     * Ensures that all red auton paths compile.
     */
    @Test
    public void compileRedPaths() {
        new PreloadedOnlyAutonSequence(swerveSubsystem, rollerSubsystem, tiltedElevatorSubsystem, true);
        new TopOnePieceAutonSequence(swerveSubsystem, rollerSubsystem, tiltedElevatorSubsystem, true);
        new TopTwoPieceAutonSequence(swerveSubsystem, rollerSubsystem, tiltedElevatorSubsystem, true);
        new BalanceAutonSequence(swerveSubsystem, rollerSubsystem, tiltedElevatorSubsystem, true);
        new BalanceAndTaxiAutonSequence(swerveSubsystem, rollerSubsystem, tiltedElevatorSubsystem, true);
        new BottomOnePieceAutonSequence(swerveSubsystem, rollerSubsystem, tiltedElevatorSubsystem, true);
        new BottomTwoPieceAutonSequence(swerveSubsystem, rollerSubsystem, tiltedElevatorSubsystem, true);
        new BottomBalanceAutonSequence(swerveSubsystem, rollerSubsystem, tiltedElevatorSubsystem, true);
    }

    /**
     * Ensures that all blue auton paths compile.
     */
    @Test
    public void compileBluePaths() {
        new PreloadedOnlyAutonSequence(swerveSubsystem, rollerSubsystem, tiltedElevatorSubsystem, false);
        new TopOnePieceAutonSequence(swerveSubsystem, rollerSubsystem, tiltedElevatorSubsystem, false);
        new TopTwoPieceAutonSequence(swerveSubsystem, rollerSubsystem, tiltedElevatorSubsystem, false);
        new BalanceAutonSequence(swerveSubsystem, rollerSubsystem, tiltedElevatorSubsystem, false);
        new BalanceAndTaxiAutonSequence(swerveSubsystem, rollerSubsystem, tiltedElevatorSubsystem, false);
        new BottomOnePieceAutonSequence(swerveSubsystem, rollerSubsystem, tiltedElevatorSubsystem, false);
        new BottomTwoPieceAutonSequence(swerveSubsystem, rollerSubsystem, tiltedElevatorSubsystem, false);
        new BottomBalanceAutonSequence(swerveSubsystem, rollerSubsystem, tiltedElevatorSubsystem, false);
    }
}
