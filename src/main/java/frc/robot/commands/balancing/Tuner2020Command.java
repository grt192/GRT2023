package frc.robot.commands.balancing;

import edu.wpi.first.wpilibj.Timer;

import com.neilalexander.jnacl.crypto.verify_16;

import edu.wpi.first.math.controller.PIDController;

import frc.robot.subsystems.drivetrain.BaseDrivetrain;
import frc.robot.subsystems.drivetrain.BaseSwerveSubsystem;

public class Tuner2020Command extends BaseBalancerCommand {
    private final PIDController drivePID;
    private final PIDController turnPID;
    private final Timer timer;

    private double returnDrivePower; // drive power to be returned to DT
    private double returnAngularPower; // angular power to return to DT (for heading correction)

    private boolean r1;
    private boolean r2;
    private boolean r3;
    private boolean r4;

    private boolean waiting; 

    private double initialHeading;
    private double oldPitch;
    private double currentPitch;
    private double deltaAngle;

    public Tuner2020Command(BaseDrivetrain driveSubsystem) {
        super(driveSubsystem);

        drivePID = new PIDController(0.3 / 35, 0.0, 0.0); // no deriv successful
        turnPID = new PIDController(0.1 / 5,0.0, 0.0); // kP = max pwr / max err
        timer = new Timer();
    }

    @Override
    public void initialize() {
        System.out.println("------------------- Balancer initialized -------------------");
        initialHeading = ahrs.getCompassHeading();
        r1 = false;
        r2 = false;
        r3 = false;
        r4 = false;
    }

    @Override
    public void execute() {
        
        if(!r1){
            if(!waiting){
                timer.reset();
                timer.start();
                waiting = true;
            }
            else{
                if(timer.hasElapsed(0.4)){
                    r1 = true;
                }
                else{
                    returnDrivePower = 0.2;
                }
            }

        }

        if(r1 && !r2){
            if(!waiting){
                timer.reset();
                timer.start();
                waiting = true;
            }
            else{
                if(timer.hasElapsed(0.4)){
                    r1 = true;
                }
                else{
                    returnDrivePower = -0.2;
                }
            }

        }

        if(r1 && r2 && !r3){
            if(!waiting){
                timer.reset();
                timer.start();
                waiting = true;
            }
            else{
                if(timer.hasElapsed(0.4)){
                    r1 = true;
                }
                else{
                    returnDrivePower = 0.2;
                }
            }

        }

        if(r1 && r2 && r3 && !r4){
            if(!waiting){
                timer.reset();
                timer.start();
                waiting = true;
            }
            else{
                if(timer.hasElapsed(0.4)){
                    r1 = true;
                }
                else{
                    returnDrivePower = 0.2;
                }
            }

        }
        

        if (driveSubsystem instanceof BaseSwerveSubsystem) {
            ((BaseSwerveSubsystem) driveSubsystem).setDrivePowers(returnDrivePower, 0.0, 0.0, true);
        } else {
            driveSubsystem.setDrivePowers(returnDrivePower);
        }
    
    }

    @Override
    public boolean isFinished() {
        return r1 && r2 && r3 && r4;
        // return reachedStation && Math.abs(ahrs.getPitch()) <= 2.0 && stoptimer.hasElapsed(0.20);
    }

    double getPower(){
        return returnDrivePower;
    }
    double getPitch(){
        return ahrs.getPitch();
    }
}
