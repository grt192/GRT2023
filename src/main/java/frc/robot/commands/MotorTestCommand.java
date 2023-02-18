package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;

import frc.robot.subsystems.RollerSubsystem;
import frc.robot.subsystems.TiltedElevatorSubsystem;
import frc.robot.subsystems.TiltedElevatorSubsystem.ElevatorState;
import frc.robot.subsystems.drivetrain.BaseSwerveSubsystem;

public class MotorTestCommand extends InstantCommand {
    private static final double TEST_DELAY_SECS = 1;
    private static final double MECH_DELAY_SECS = 2;
    private static final double SWERVE_DELAY_SECS = 2;

    public MotorTestCommand(BaseSwerveSubsystem swerveSubsystem, TiltedElevatorSubsystem tiltedElevatorSubsystem, RollerSubsystem rollerSubsystem) {
        super(() -> {
            // Run tilted elevator to all heights
            // GROUND - CHUTE - SUBSTATION - CUBEMID - CUBEHIGH - CONEMID - CONEHIGH - SUBSTATION
            tiltedElevatorSubsystem.setState(ElevatorState.GROUND);
            Timer.delay(TEST_DELAY_SECS);

            tiltedElevatorSubsystem.setState(ElevatorState.CHUTE);
            Timer.delay(TEST_DELAY_SECS);

            tiltedElevatorSubsystem.setState(ElevatorState.SUBSTATION);
            Timer.delay(TEST_DELAY_SECS);

            tiltedElevatorSubsystem.setState(ElevatorState.CUBE_MID);
            Timer.delay(TEST_DELAY_SECS);

            tiltedElevatorSubsystem.setState(ElevatorState.CUBE_HIGH);
            Timer.delay(TEST_DELAY_SECS);

            tiltedElevatorSubsystem.setState(ElevatorState.CONE_MID);
            Timer.delay(TEST_DELAY_SECS);

            tiltedElevatorSubsystem.setState(ElevatorState.CONE_HIGH);
            Timer.delay(TEST_DELAY_SECS);

            tiltedElevatorSubsystem.setState(ElevatorState.SUBSTATION);
            Timer.delay(MECH_DELAY_SECS);

            // Run roller in and out
            for (double i = -1 ; i <= 1; i += .1){
                rollerSubsystem.setRollPower(i);
                Timer.delay(0.2);
            }
            Timer.delay(TEST_DELAY_SECS);

            // Open motor
            rollerSubsystem.openMotor();
            Timer.delay(MECH_DELAY_SECS);

            /*
            //swerve stuff
            //forwards
            swerveSubsystem.setDrivePowers(1, 0, 0, false);
            Timer.delay(SWERVE_DELAY_SECS);
            //backwards
            swerveSubsystem.setDrivePowers(-1, 0, 0, false);
            Timer.delay(SWERVE_DELAY_SECS);
            //left
            swerveSubsystem.setDrivePowers(0, 1, 0, false);
            Timer.delay(SWERVE_DELAY_SECS);
            //right
            swerveSubsystem.setDrivePowers(0, -1, 0, false);
            Timer.delay(SWERVE_DELAY_SECS);
            //turn left
            swerveSubsystem.setDrivePowers(0, 0, 1, false);
            Timer.delay(SWERVE_DELAY_SECS);
            //turn right
            swerveSubsystem.setDrivePowers(0, 0, -1, false);
            */
        }, swerveSubsystem, tiltedElevatorSubsystem, rollerSubsystem);

        withInterruptBehavior(InterruptionBehavior.kCancelIncoming);
    }
}
