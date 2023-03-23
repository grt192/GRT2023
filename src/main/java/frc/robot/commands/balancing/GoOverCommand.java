package frc.robot.commands.balancing;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.subsystems.drivetrain.BaseDrivetrain;
import frc.robot.subsystems.drivetrain.BaseSwerveSubsystem;

/**
 * A command that moves robot over charging station at a varying speed, 
 *  changing speed based on the stage of the charging station the robot is on. 
 */
public class GoOverCommand extends CommandBase {
    private final double APPROACH_STATION_POWER = -0.75;
    private final double CLIMBING_STATION_POWER = -0.45;
    private final double PAST_CENTER_POWER = -0.25;

    private final BaseDrivetrain driveSubsystem;
    private final AHRS ahrs; 

    private double returnDrivePower;
    private double targetHeading;
    private double currentPitch;

    private boolean reachedStation;
    private boolean passedCenter;
    private boolean overStation;
    private boolean waiting;
    private double POWER_SCALE = 0.5;

    private final Timer timer;

    public GoOverCommand(BaseDrivetrain driveSubsystem, boolean isRed) {
        this.driveSubsystem = driveSubsystem;
        this.ahrs = driveSubsystem.getAhrs();
        this.targetHeading = isRed
            ? 0.0
            : Math.PI;
        timer = new Timer();
        addRequirements(driveSubsystem);
    }

    @Override
    public void initialize() {
        reachedStation = false;
        passedCenter = false;
        overStation = false;
        waiting = false;
    }

    @Override
    public void execute() {
        currentPitch = ahrs.getPitch();
        System.out.println("Pitch" + currentPitch);

        if (!reachedStation) {
            returnDrivePower = APPROACH_STATION_POWER;
            reachedStation = currentPitch <= -7.0;
            if(reachedStation) System.out.println("reached station");
        }

        if (reachedStation && !passedCenter) {
            returnDrivePower = CLIMBING_STATION_POWER;
            if (currentPitch >= -7.0){
                passedCenter = true;
                System.out.println("passed center");
            } 
        }
        if (reachedStation && passedCenter && !overStation) {
            if (Math.abs(currentPitch) <= 2.0) {
                overStation = true;
                System.out.println("over station");
            }
            else returnDrivePower = PAST_CENTER_POWER;
        }

        if (overStation && !waiting) {
            timer.reset();
            timer.start();
            returnDrivePower = PAST_CENTER_POWER;
            waiting = true;
            System.out.println("waiting");
        }

        if(driveSubsystem instanceof BaseSwerveSubsystem){
            ((BaseSwerveSubsystem)driveSubsystem).setDrivePowersWithHeadingLock(returnDrivePower * POWER_SCALE,0.0, Rotation2d.fromRadians(targetHeading), true);
        } 
        else {
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
        return overStation && timer.hasElapsed(2.0);
    }
}
