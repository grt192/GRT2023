package frc.robot.commands;

import com.fasterxml.jackson.databind.JsonSerializable.Base;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.subsystems.drivetrain.BaseDrivetrain;
import frc.robot.subsystems.drivetrain.BaseSwerveSubsystem;
import edu.wpi.first.wpilibj.Timer;

public class BalancerCommand extends CommandBase {
    private final BaseDrivetrain driveSubsystem;
    private final AHRS ahrs; 
    
    private double initialHeading;
    private double oldAngle;
    private double currentAngle;
    private double angularAcceleration;

    PIDController drivePID;
    PIDController turnPID;

    private double returnDrivePower; // drive power to be returned to DT
    private double returnAngularPower; // angular power to return to DT (for heading correction)

    public boolean reachedStation;
    public boolean passedCenter;

    private final Timer timer;

    public BalancerCommand(BaseDrivetrain driveSubsystem) {
        this.driveSubsystem = driveSubsystem;
        this.ahrs = driveSubsystem.getAhrs();

        drivePID = new PIDController(0.3/35, 0.0, 0.0); // no deriv successful
        turnPID = new PIDController(0.2/5,0.0, 0.0); // kP = max pwr / max err
        timer = new Timer();
        addRequirements(driveSubsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        System.out.println("------------------- Balancer initialized -------------------");
        initialHeading = ahrs.getCompassHeading();
        reachedStation = false;

    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        
        returnAngularPower = turnPID.calculate((initialHeading - ahrs.getCompassHeading()), 0); // correct angle of approach
        
        if(!reachedStation) {
            returnDrivePower = 0.80;
            if(ahrs.getPitch() >= 15.0) reachedStation = true;
        }
        else{
            currentAngle = ahrs.getPitch();
            angularAcceleration = Math.abs(currentAngle - oldAngle) / timer.get(); // calc magnitude of angular acceleration based on delta angle over time
            timer.reset();

            if(!passedCenter){
                returnDrivePower = 0.15; //.15 successful
                if(ahrs.getPitch() <= 0.0){
                    passedCenter = true; // <= 1.0 worked 
                    if(driveSubsystem instanceof BaseSwerveSubsystem) ((BaseSwerveSubsystem) driveSubsystem).lockNow();
                    // lock the moment the CG passes the center to minimize overshoot
                }
            }
            else{
                if(Math.abs(ahrs.getPitch()) <= 2.0){
                    returnDrivePower = 0.0;
                    if(driveSubsystem instanceof BaseSwerveSubsystem) ((BaseSwerveSubsystem) driveSubsystem).lockNow();
                    // lock as soon as level is detected
                }
                else{
                    if(angularAcceleration <= 0.3){ // threshold value (deg / sec) TBD
                        returnDrivePower = -1 * drivePID.calculate(ahrs.getPitch(), 0);
                        // if acceleration is low, platform is relatively stable, so try leveling
                    }
                    else{
                        returnDrivePower = 0.0;
                        if(driveSubsystem instanceof BaseSwerveSubsystem) ((BaseSwerveSubsystem) driveSubsystem).lockNow();
                        // if platform is rocking, lock wheels to preserve stability
                    }
                }
            }
        }

        if(driveSubsystem instanceof BaseSwerveSubsystem){
            ((BaseSwerveSubsystem) driveSubsystem).setDrivePowers(returnDrivePower, returnAngularPower);
        }
        else driveSubsystem.setDrivePowers(returnDrivePower);

        oldAngle = currentAngle; // set the current angle to old angle so it is accessible for next cycle     
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        System.out.println("------------------- Balancing process finished -------------------");
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return reachedStation && Math.abs(ahrs.getPitch()) <= 2.0;
        // return reachedStation && Math.abs(ahrs.getPitch()) <= 2.0 && stoptimer.hasElapsed(0.20);
    }
}
