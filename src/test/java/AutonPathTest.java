import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import static org.junit.jupiter.api.Assertions.assertEquals;

import edu.wpi.first.hal.HAL;

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
    private SwerveSubsystem swerveSubsystem;
    private RollerSubsystem rollerSubsystem;
    private TiltedElevatorSubsystem tiltedElevatorSubsystem;

    @BeforeEach
    public void setup() {
        assert HAL.initialize(500, 0); // initialize the HAL, crash if failed

        swerveSubsystem = new SwerveSubsystem(null);
        // swerveSubsystem.getAhrs().enableLogging(false);
        rollerSubsystem = new RollerSubsystem();
        tiltedElevatorSubsystem = new TiltedElevatorSubsystem();
    }

    @AfterEach
    public void shutdown() throws Exception {
        swerveSubsystem.close();
        rollerSubsystem.close();
        tiltedElevatorSubsystem.close();
    }

    @Test
    public void compileTestPaths() {
        new StraightLinePath(swerveSubsystem);
        new HighRotationLinePath(swerveSubsystem);
        new RotatingSCurveAutonSequence(swerveSubsystem);
        new BoxAutonSequence(swerveSubsystem);
        new GRTAutonSequence(swerveSubsystem);
    }

    @Test
    public void compileRedPaths() {
        new RedTopAuton(swerveSubsystem, rollerSubsystem, tiltedElevatorSubsystem);
        new RedBalanceAuton(swerveSubsystem, rollerSubsystem, tiltedElevatorSubsystem);
        new RedBottomAuton(swerveSubsystem, rollerSubsystem, tiltedElevatorSubsystem);
    }

    @Test
    public void compileBluePaths() {
        new BlueTopAuton(swerveSubsystem, rollerSubsystem, tiltedElevatorSubsystem);
        // new BlueBalanceAuton(swerveSubsystem, rollerSubsystem, tiltedElevatorSubsystem);
        new BlueBottomAuton(swerveSubsystem, rollerSubsystem, tiltedElevatorSubsystem);
    }
}
