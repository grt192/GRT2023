package frc.robot.commands.Balancing;

import com.fasterxml.jackson.databind.JsonSerializable.Base;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.subsystems.drivetrain.BaseDrivetrain;
import frc.robot.subsystems.drivetrain.BaseSwerveSubsystem;
import edu.wpi.first.wpilibj.Timer;

public class DefaultBalancerCommand extends CommandBase {
    private final BaseDrivetrain driveSubsystem;
    private final AHRS ahrs; 
    
    private double initialHeading;
    private double oldPitch;
    private double currentPitch;
    private double deltaAngle;

    PIDController drivePID;
    PIDController turnPID;

    private double returnDrivePower; // drive power to be returned to DT
    private double returnAngularPower; // angular power to return to DT (for heading correction)

    public boolean reachedStation;
    public boolean passedCenter;
    public boolean waiting;
    public boolean balanced;

    private final Timer timer;

    public DefaultBalancerCommand(BaseDrivetrain driveSubsystem) {
        this.driveSubsystem = driveSubsystem;
        this.ahrs = driveSubsystem.getAhrs();

        drivePID = new PIDController(0.3/35, 0.0, 0.0); // no deriv successful
        turnPID = new PIDController(0.1/5,0.0, 0.0); // kP = max pwr / max err
        timer = new Timer();
        addRequirements(driveSubsystem);
    }

    @Override
    public void initialize() {
        System.out.println("------------------- Balancer initialized -------------------");
        initialHeading = ahrs.getCompassHeading();
        reachedStation = false;
        passedCenter = false;
        balanced = false;
        waiting = false;
    }

    @Override
    public void execute() {
        
        returnAngularPower = turnPID.calculate(ahrs.getCompassHeading(), initialHeading); // correct angle of approach
        
        if(!reachedStation) {
            returnDrivePower = - 0.80;
            if(ahrs.getPitch() <= - 15.0){
                reachedStation = true;
                returnDrivePower = - 0.2;
            } //.15 successful
        } else {
            currentPitch = ahrs.getPitch();
            deltaAngle = Math.abs(currentPitch - oldPitch); // calc magnitude of angular acceleration based on delta angle over time

            if(!passedCenter){
                if(ahrs.getPitch() >= -5.0){
                    passedCenter = true; // <= 1.0 worked 
                }
            } else {
                returnDrivePower = -1 * drivePID.calculate(ahrs.getPitch(), 0);
                System.out.println(returnDrivePower);
                if(Math.abs(ahrs.getPitch()) <= 1.0 && deltaAngle <= 0.1) balanced = true;
            }
        }

        if(driveSubsystem instanceof BaseSwerveSubsystem){
            ((BaseSwerveSubsystem) driveSubsystem).setDrivePowers(returnDrivePower, 0.0);
        } else {
            driveSubsystem.setDrivePowers(returnDrivePower);
        }

        oldPitch = currentPitch; // set the current angle to old angle so it is accessible for next cycle     
    }

    @Override
    public void end(boolean interrupted) {
        System.out.println("------------------- Balancing process finished -------------------");
        if (driveSubsystem instanceof BaseSwerveSubsystem) {
            BaseSwerveSubsystem swerve = (BaseSwerveSubsystem) driveSubsystem;
            swerve.setChargingStationLocked(true);
            swerve.lockNow();
        }
    }

    @Override
    public boolean isFinished() {
        return reachedStation && balanced;
        // return reachedStation && Math.abs(ahrs.getPitch()) <= 2.0 && stoptimer.hasElapsed(0.20);
    }
}
