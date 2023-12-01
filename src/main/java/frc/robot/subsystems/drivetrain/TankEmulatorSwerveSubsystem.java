package frc.robot.subsystems.drivetrain;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.subsystems.leds.LEDSubsystem;
import frc.robot.vision.PhotonWrapper;



public class TankEmulatorSwerveSubsystem extends SwerveSubsystem{
    
    private static final double SPEED_SCALE = .8 / 2;

    public TankEmulatorSwerveSubsystem(PhotonWrapper photonWrapper, LEDSubsystem ledSubsystem){
        super(photonWrapper,  ledSubsystem);
    }

    /**
     * Makes the swerve modules act as a tank drive
     * @param forwardPower The forward power of the robot [-1,1]
     * @param rotatePower The rotational power of the robot, clockwise positive [-1,1]
     */
    public void setDrivePowers(double forwardPower, double rotatePower){

        double leftPower = SPEED_SCALE * (forwardPower + rotatePower);
        double rightPower = SPEED_SCALE * (forwardPower - rotatePower);

        SwerveModuleState[] states = {
            new SwerveModuleState(MAX_VEL * leftPower, new Rotation2d(0)), //TL
            new SwerveModuleState(MAX_VEL * rightPower, new Rotation2d(0)), //TR
            new SwerveModuleState(MAX_VEL * leftPower, new Rotation2d(0)), //BL
            new SwerveModuleState(MAX_VEL * rightPower, new Rotation2d(0)) //BR
        };

        setSwerveModuleStates(states);
    }
}
