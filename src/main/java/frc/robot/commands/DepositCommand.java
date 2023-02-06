package frc.robot.commands;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.commands.swerve.FollowPathCommand;
import frc.robot.subsystems.drivetrain.BaseDrivetrain;
import frc.robot.subsystems.drivetrain.BaseSwerveSubsystem;

public class DepositCommand extends CommandBase {
    private final BaseSwerveSubsystem driveSubsystem; // changed to BASE SWERVE SUBSYS to allow for getRobotPosition() 
    private final AHRS ahrs; 

    private final boolean red;


    /*  all below setpoint constants are in inches
        X axis --> long axis of field
        Y axis --> short axis of field
        origin at corner of blue grid 
    */ 
    private final double BLUE_X_SET = 86.0; 
    private final double BLUE_Y_SET = 30.0;
    private final double BLUE_R_SET = -90.0; // rotation in degrees
    
    private final double RED_X_SET = 564.0; 
    private final double RED_Y_SET = 30.0;
    private final double RED_R_SET = 90.0; // rotation in degrees

    Pose2d blueSetPoint = new Pose2d(BLUE_X_SET, BLUE_Y_SET, Rotation2d.fromDegrees(BLUE_R_SET));
    Pose2d redSetPoint = new Pose2d(RED_X_SET, RED_Y_SET, Rotation2d.fromDegrees(RED_R_SET));

    private enum Stage{
        Engaged,
        OnAxis,
        Normal2Node, 
        Ready,
        Deposited
    }

    private Stage stage;
    
    public DepositCommand(BaseSwerveSubsystem driveSubsystem) { // changed to BASE SWERVE SUBSYS to allow for getRobotPosition() 
        this.driveSubsystem = driveSubsystem;
        this.ahrs = driveSubsystem.getAhrs();

        red = true;
        stage = Stage.Engaged;

        addRequirements(driveSubsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        System.out.println("---------------------- Depositor initialized ----------------------");
        
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {

        if(stage == Stage.Engaged){
            if(red){
                FollowPathCommand trajectory = new FollowPathCommand(
                    driveSubsystem, 
                    driveSubsystem.getRobotPosition(), 
                    null,
                    redSetPoint,
                    true); 
            }
            else{
                FollowPathCommand trajectory = new FollowPathCommand(
                    driveSubsystem, 
                    driveSubsystem.getRobotPosition(), 
                    null,
                    blueSetPoint,
                    true); 
            }
            
            // insert run trajectory 

        }

        if(stage == Stage.OnAxis){ 

            // driver input turned into pose2d along Y axis that robot goes to to deposit

            // create next trajectory to that spot
        }

        if(stage == Stage.Normal2Node){

            // correct angle to multiple of 90º

            // final vision-based alignment maybe?
        }

        if(stage == Stage.Ready){
            // insert appropriate command based on selected deposit spot
            // deposit
        }

        if(stage == Stage.Deposited){
            // reset variables and robot and hand control back to driver
        }
        
        driveSubsystem.setDrivePowers(0); //placeholder
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        System.out.println("----------- Balancing process finished ---------------------");
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false; // placeholder
    }
}
