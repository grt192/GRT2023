package frc.robot.commands;

import javax.management.relation.Role;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.RollerSubsystem;
import frc.robot.subsystems.TiltedElevatorSubsystem;
import frc.robot.subsystems.TiltedElevatorSubsystem.ElevatorState;
import frc.robot.subsystems.drivetrain.BaseSwerveSubsystem;

public class MotorTest extends CommandBase {
    private BaseSwerveSubsystem swerveSubsystem;
    private TiltedElevatorSubsystem tiltedElevatorSubsystem;
    private RollerSubsystem rollerSubsystem;

    private final int betweenDelay = 1;
    private final int betweenMechDelay = 2;
    public MotorTest(BaseSwerveSubsystem swerveSubsystem, TiltedElevatorSubsystem tiltedElevatorSubsystem, RollerSubsystem rollerSubsystem){
        CommandScheduler.getInstance().cancelAll();
        this.swerveSubsystem = swerveSubsystem;
        this.tiltedElevatorSubsystem = tiltedElevatorSubsystem;
        this.rollerSubsystem = rollerSubsystem;
    }
    @Override
    public void initialize() {
        //Run tilted elevator to all heights
        //GROUND - CHUTE - SUBSTATION - CUBEMID - CUBEHIGH - CONEMID - CONEHIGH - GROUND
        tiltedElevatorSubsystem.setState(ElevatorState.GROUND);
        Timer.delay(betweenDelay);
        tiltedElevatorSubsystem.setState(ElevatorState.CHUTE);
        Timer.delay(betweenDelay);
        tiltedElevatorSubsystem.setState(ElevatorState.SUBSTATION);
        Timer.delay(betweenDelay);
        tiltedElevatorSubsystem.setState(ElevatorState.CUBE_MID);
        Timer.delay(betweenDelay);
        tiltedElevatorSubsystem.setState(ElevatorState.CUBE_HIGH);
        Timer.delay(betweenDelay);
        tiltedElevatorSubsystem.setState(ElevatorState.CONE_MID);
        Timer.delay(betweenDelay);
        tiltedElevatorSubsystem.setState(ElevatorState.CONE_HIGH);
        Timer.delay(betweenDelay);
        tiltedElevatorSubsystem.setState(ElevatorState.GROUND);

        Timer.delay(betweenMechDelay);

        //run roller in and out
        for (double i = -1 ; i <= 1; i += .1){
            rollerSubsystem.setRollPower(i);
            Timer.delay(.2);
        }
        Timer.delay(betweenDelay);
        //run openmotor
        rollerSubsystem.openMotor();

        Timer.delay(betweenMechDelay);
        
        //swerve stuff
        //forwards
        swerveSubsystem.setDrivePowers(1, 0, 0, false);
        Timer.delay(betweenDelay);
        //backwards
        swerveSubsystem.setDrivePowers(-1, 0, 0, false);
        Timer.delay(betweenDelay);
        //left
        swerveSubsystem.setDrivePowers(0, 1, 0, false);
        Timer.delay(betweenDelay);
        //right
        swerveSubsystem.setDrivePowers(0, -1, 0, false);
        Timer.delay(betweenDelay);
        //turn left
        swerveSubsystem.setDrivePowers(0, 0, 1, false);
        Timer.delay(betweenDelay);
        //turn right
        swerveSubsystem.setDrivePowers(0, 0, -1, false);    
    }

    @Override
    public boolean isFinished() {
        return true;
    }

    @Override
    public void end(boolean interrupted) {
        System.out.print("uharw");
    }
}
