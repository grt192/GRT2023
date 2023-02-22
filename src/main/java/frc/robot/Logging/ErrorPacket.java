package frc.robot.Logging;

import edu.wpi.first.math.geometry.Pose2d;

public class ErrorPacket implements Message{
    
    private Pose2d position;
    private String m;
    private double velocity;

    ErrorPacket(String m, Pose2d position, double velocity){
        this.position = position;
        this.velocity = velocity;
        this.m = m;
    }

    @Override
    public String display(){
        return "[ERROR] Message: " + m + " Position: (" + position.getX() + ", " + position.getY() + ") Heading: " + position.getRotation().getDegrees();
    }

}

