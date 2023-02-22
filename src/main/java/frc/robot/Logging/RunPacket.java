package frc.robot.Logging;

import edu.wpi.first.math.geometry.Pose2d;

public class RunPacket implements Message{
    
    private Pose2d position;
    private double velocity;

    RunPacket(Pose2d position, double velocity){
        this.position = position;
        this.velocity = velocity;
    }

    @Override
    public String display(){
        return "[RUN] " + "Position: (" + position.getX() + ", " + position.getY() + ") Heading: " + position.getRotation().getDegrees();
    }

}

