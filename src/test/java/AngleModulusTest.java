import org.junit.*;
import static org.junit.Assert.*;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;

import frc.robot.subsystems.swerve.SwerveModule;

public class AngleModulusTest {
    /**
     * This test ensures that the optimize method is able to correctly perform angle
     * wraparound (going from pi to 7pi/6 when the target angle is -5pi/6) without
     * optimization; in this case the delta should be less than pi/2.
     */
    @Test
    public void unoptimizedWraparound() {
        SwerveModuleState target = new SwerveModuleState(1.0, new Rotation2d(-5 * Math.PI / 6.0));
        var optimized = SwerveModule.optimizeModuleState(target, new Rotation2d(29 * Math.PI / 6.0));

        // Velocity: 1.0 (not flipped), angle: 29pi/6 + pi/3 (delta)
        assertEquals(1.0, optimized.getFirst(), 0);
        assertEquals(31 * Math.PI / 6.0, optimized.getSecond(), 0);
    }

    /**
     * This test ensures that the optimize method is able to correctly perform angle
     * wraparound (going from pi to 7pi/6 when the target angle is -5pi/6) while also performing
     * optimization; in this case the delta should be greater than pi/2.
     */
    @Test
    public void optimizedWraparound() {
        SwerveModuleState target = new SwerveModuleState(1.0, new Rotation2d(-5 * Math.PI / 6.0));
        var optimized = SwerveModule.optimizeModuleState(target, new Rotation2d(26 * Math.PI / 6.0));

        // Velocity: -1.0 (flipped), angle: 26pi/6 - pi/6 (delta)
        assertEquals(-1.0, optimized.getFirst(), 0);
        assertEquals(25 * Math.PI / 6.0, optimized.getSecond(), 0);
    }
}
