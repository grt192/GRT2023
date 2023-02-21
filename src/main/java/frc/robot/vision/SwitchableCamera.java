package frc.robot.vision;

import org.opencv.core.Core;
import org.opencv.core.Mat;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.CvSink;
import edu.wpi.first.cscore.CvSource;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.cscore.VideoSource;
import edu.wpi.first.wpilibj.DriverStation;

/**
 * A `VideoSource` that toggles between a front and back camera.
 */
public class SwitchableCamera {
    private final UsbCamera topCam;
    private final UsbCamera bottomCam;

    private final int WIDTH = 140;
    private final int HEIGHT = 120;

    private final CvSink bottomSink;
    private final CvSink topSink;

    private final CvSource server;

    private volatile boolean frontActive = true;

    public SwitchableCamera() {
        topCam = new UsbCamera("top cam", 0);
        bottomCam = new UsbCamera("bot cam", 1);

        topCam.setResolution(WIDTH, HEIGHT);
        bottomCam.setResolution(WIDTH, HEIGHT);

        topCam.setFPS(30);
        bottomCam.setFPS(30);

        bottomSink = new CvSink("flip bottom cam");
        bottomSink.setSource(bottomCam);

        topSink = new CvSink("flip bottom cam");
        topSink.setSource(topCam);

        this.server = CameraServer.putVideo("test", HEIGHT, WIDTH);

        Runnable stream = () -> {
            Mat img = new Mat();
            while (!Thread.interrupted()) {
                if (frontActive) {
                    if (bottomSink.grabFrame(img) != 0) {
                        Core.rotate(img, img, Core.ROTATE_90_CLOCKWISE);
                        server.putFrame(img);
                    } else {
                        DriverStation.reportError(bottomSink.getError(), false);
                    }
                } else {
                    if (topSink.grabFrame(img) != 0) {
                        server.putFrame(img);
                    } else {
                        DriverStation.reportError(topSink.getError(), false);
                    }
                }

            }
        };

        Thread flipThread = new Thread(stream);
        flipThread.setDaemon(true);
        flipThread.start();
    }

    /**
     * Switches the source connected to this camera (toggles between front and
     * back).
     */
    public void switchCamera() {
        frontActive = !frontActive;
    }

    /**
     * Gets the `VideoSource` of the camera.
     * 
     * @return The connected `VideoSource`.
     */
    public VideoSource getSource() {
        return server;
    }
}
