package frc.robot.jetson;

/**
 * The JSON data object sent by the jetson.
 */
public class JetsonData {
    // `Translation3d` in meters of the target relative to the camera.
    public double x;
    public double y;
    public double z;

    // `Rotation3d` in quaternion form of the target relative to the camera.
    public double qw;
    public double qx;
    public double qy;
    public double qz;

    // The timestamp of the camera frame.
    public long ts;

    // The id of the detected target. 0-29 for an AprilTag, and 30+ for a custom vision target.
    public int tid;
    public int cid;
}
