package frc.robot.commands.balancing;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DataLogEntry;
import edu.wpi.first.util.datalog.StringLogEntry;

import frc.robot.subsystems.drivetrain.BaseDrivetrain;
import frc.robot.subsystems.drivetrain.BaseSwerveSubsystem;

import static frc.robot.Constants.BalancerConstants.*;

public class DefaultBalancerCommand extends BaseBalancerCommand {
    private final PIDController drivePID;

    private final Timer runawayTimer;

    private double returnDrivePower; // drive power to be returned to DT
    private double prevPitchDegs;
    
    private final double direction;
    private final boolean reverse;
    private Rotation2d targetHeading;

    private boolean reachedStation;
    private boolean passedCenter;
    private boolean balanced;
    private boolean runaway;

    private final StringLogEntry balanceLog;

    /**
     * Constructs a default balancer command from a drive subsystem and a boolean indicating
     * whether to balance from the front or back.
     * 
     * This command was used at all competitions in 2023. 
     * 
     * @param driveSubsystem The drive subsystem.
     * @param reverseBalance true if the robot balances driving backwards
     */
    public DefaultBalancerCommand(BaseDrivetrain driveSubsystem, boolean reverseBalance) {
        super(driveSubsystem);

        // value of Kp tuned to competition charging station values at the 2023 Utah Regional
        // Attempts were made to alter this constant in pursuit of a faster balance time. 
        drivePID = new PIDController(COMP_CHARGING_STATION_KP, 0.0, 0.0);

        runawayTimer = new Timer();

        balanceLog = new StringLogEntry(DataLogManager.getLog(), "balanceLog");

        // by default, the robot balances driving backwards (battery side first)
        // this kept the center of mass further forward and increased control and balancing performance
        // balancing can be performed driving forwards (e.g. if we need to intake from the center of the field)
        direction = reverseBalance ? 1 : -1;
        reverse = reverseBalance;
    }

    @Override
    public void initialize() {
        System.out.println("------------------- Balancer initialized -------------------");
        balanceLog.append("Balancer Initialized");

        // global state initialization
        reachedStation = false;
        passedCenter = false;
        balanced = false;
        runaway = false;

        // set the heading lock to ensure the robot stays straight while balancing. 
        // the compass has already been initialized to the field directions.
        if (driveSubsystem instanceof BaseSwerveSubsystem) {
            double currentHeadingRads = ((BaseSwerveSubsystem) driveSubsystem).getDriverHeading().getRadians();
            double lockHeadingRads = (Math.abs(currentHeadingRads) > Math.PI / 2.0) ? Math.PI : 0;

            targetHeading = new Rotation2d(lockHeadingRads);
        } else {
            targetHeading = new Rotation2d();
        }

        // timers ensure that robot does not run away at full power
        runawayTimer.reset();
        runawayTimer.start();
    }

    @Override
    public void execute() {
        double currentPitchDegs = ahrs.getPitch();

        if (!reachedStation) {
            returnDrivePower = -0.75 * direction;

            // if the robot seems to have run away (see 2023 Utah Regional Playoff Match)
            // make the balancer stop so we don't get a bunch of penalty points
            if ((reverse && currentPitchDegs <= -10.0) || (!reverse && currentPitchDegs >= 10.0)) {
                reachedStation = true;
                balanceLog.append("Reached Charging Station");
                returnDrivePower = -0.17 * direction;
            } else if (runawayTimer.hasElapsed(2.0)) {
                returnDrivePower = 0.0;
                System.out.println("BALANCER RUNAWAY DETECTED");
                balanceLog.append("BALANCER RUNAWAY DETECTED - BALANCING INTERRUPTED");
                runaway = true;
            }
        } else {
            double deltaPitchDegs = Math.abs(currentPitchDegs - prevPitchDegs); 

            // passing the center of the charging station is the moment the center of mass of the robot passes the fulcrum of the charging station
            // this is useful to know when the robot is fully on the charging station, as starting PID balancing while the robot's front wheels are stuck in the hinge could lead to failure
            if (!passedCenter) {
                if ((reverse && currentPitchDegs >= -10.0) || (!reverse && currentPitchDegs <= 10.0)) { // reverse && currentPitchDegs >= -8.0
                    passedCenter = true;
                    balanceLog.append("Passed Center of Charging Station");
                }
            } else {
                returnDrivePower = -1 * drivePID.calculate(currentPitchDegs, 0);
                // if ((reverse && Math.abs(currentPitchDegs) <= 1.0 && deltaPitchDegs <= 0.1) || (!reverse && Math.abs(currentPitchDegs) >= -1.0 && deltaPitchDegs >= -0.1)){
                // gauge whether robot is balanced or not by measuring robot pitch as well as the delta pitch
                    if (Math.abs(currentPitchDegs) <= 1.0 && ((reverse && deltaPitchDegs <= 0.1) || (!reverse && deltaPitchDegs >= -0.1))){
                    balanced = true;
                    balanceLog.append("Robot balanced");
                }
            }
        }

        // if AHRS unit is disconnected, stop the balancer and log event (see 2023 Utah Regional, again)
        if (!ahrs.isConnected()) {
            returnDrivePower = 0.0;
            System.out.println("BALANCER RUNAWAY DETECTED");
            balanceLog.append("NAVX DISCONNECT DETECTED - BALANCING INTERRUPTED");
            runaway = true;            
        }

        
        // account for the different drive subsystems command might use
        if (driveSubsystem instanceof BaseSwerveSubsystem) {
            BaseSwerveSubsystem swerveSubsystem = (BaseSwerveSubsystem) driveSubsystem;
            swerveSubsystem.setDrivePowersWithHeadingLock(
                returnDrivePower,
                0.0,
                targetHeading,
                true
            );
        } else {
            driveSubsystem.setDrivePowers(returnDrivePower);
        }

        // maintain the previous pitch value to calculate the running derivative
        prevPitchDegs = currentPitchDegs;
    }

    @Override
    public boolean isFinished() {
        // stop the robot if its become level since reaching the charging station
        // or if it's likely that the robot has run away
        return (reachedStation && balanced) || runaway;
    }
}
