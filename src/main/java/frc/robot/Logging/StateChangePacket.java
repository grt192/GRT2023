package frc.robot.Logging;

import edu.wpi.first.math.geometry.Pose2d;

public class StateChangePacket implements Message{

    private Pose2d position;
    private double velocity;
    private String state1;
    private String state2;

    StateChangePacket( String state1, String state2, Pose2d position, double velocity){
        this.position = position;
        this.velocity = velocity;
        this.state1 = state1;
        this.state2 = state2;
    }

    @Override
    public String display(){
        return "[STATE] " + state1 + " --> " + state2 + " Position: (" + position.getX() + ", " + position.getY() + ") Heading: " + position.getRotation().getDegrees();
    }

}

