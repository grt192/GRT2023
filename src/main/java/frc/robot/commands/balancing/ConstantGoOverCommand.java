package frc.robot.commands.balancing;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.subsystems.drivetrain.BaseDrivetrain;
import frc.robot.subsystems.drivetrain.BaseSwerveSubsystem;

/**
 * A command that moves robot over charging station at a constant speed, 
 * pausing after passing the center of rotation to allow charging station motion,
 * and then continuing on to exit the community.
 */
public class ConstantGoOverCommand extends CommandBase {
    private final BaseDrivetrain driveSubsystem;
    private final AHRS ahrs; 

    private double returnDrivePower;
    private double targetHeading;

    private boolean overStation;
    private boolean waiting;
    private boolean waited;

    private static final double POWER_SCALE = 0.5;

    private final Timer timer;

    public ConstantGoOverCommand(BaseDrivetrain driveSubsystem, boolean isRed) {
        this.driveSubsystem = driveSubsystem;
        this.ahrs = driveSubsystem.getAhrs();

        this.targetHeading = isRed
            ? Math.PI
            : 0.0;

        timer = new Timer();
        addRequirements(driveSubsystem);
    }

    @Override
    public void initialize() {
        overStation = false;
        waiting = false;
        waited = false; 
    }

    @Override
    public void execute() {
        double currentPitch = ahrs.getPitch();
        System.out.println("Pitch" + currentPitch);

        if (!waited && currentPitch >= 2.0) { // if we haven't already waited
            if (!waiting) { // if we need to wait, and we are not currently waiting
                timer.reset();
                timer.start();
                waiting = true;
                returnDrivePower = 0.0;
            }
            if (waiting && timer.hasElapsed(0.25)) { // if we are waiting and it has been at least 0.25 seconds
                waited = true;
                waiting = false;
                returnDrivePower = -0.5;
                overStation = true;
                timer.reset();
                timer.start();
            }
        } else {
            returnDrivePower = -0.6; // if we do not need to wait and are not waiting
        }

        if (driveSubsystem instanceof BaseSwerveSubsystem) {
            ((BaseSwerveSubsystem) driveSubsystem).setDrivePowersWithHeadingLock(
                returnDrivePower * POWER_SCALE,
                0.0,
                Rotation2d.fromRadians(targetHeading),
                true
            );
        } else {
            driveSubsystem.setDrivePowers(returnDrivePower * POWER_SCALE);
        }
    }

    @Override 
    public void end(boolean interrupted){
        driveSubsystem.setDrivePowers(0.0);
        timer.reset();
        System.out.println("ended");
    }

    @Override
    public boolean isFinished() {
        return overStation && timer.hasElapsed(1.5);
    }
}
