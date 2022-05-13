package frc.robot.shuffleboard;

import static frc.robot.Constants.ShuffleboardConstants.UPDATE_TIME;

import java.util.ArrayList;

import edu.wpi.first.wpilibj.Timer;

public class ShuffleboardManager {
    // singleton so that we can start the thread automatically whenever we
    // register a network table entry
    private static ShuffleboardManager theManager;

    private final ArrayList<GRTNetworkTableEntry> entries = new ArrayList<>();

    public static void registerEntry(GRTNetworkTableEntry entry) {
        ShuffleboardManager instance = getInstance();
        synchronized (instance.entries) {
            instance.entries.add(entry);
        }
    }

    public static void removeEntry(GRTNetworkTableEntry entry) {
        ShuffleboardManager instance = getInstance();
        synchronized (instance.entries) {
            instance.entries.remove(entry);
        }
    }

    private static ShuffleboardManager getInstance() {
        if (theManager == null) {
            theManager = new ShuffleboardManager();
        }

        return theManager;
    }

    public ShuffleboardManager() {
        ShuffleboardRunnable runnable = new ShuffleboardRunnable();

        Thread thread = new Thread(runnable);
        thread.setDaemon(true);
        thread.start();
    }

    class ShuffleboardRunnable implements Runnable {
        @Override
        public void run() {
            while (true) {
                double nextLoop = Timer.getFPGATimestamp() + UPDATE_TIME;

                synchronized (entries) {
                    for (GRTNetworkTableEntry entry : entries) {
                        entry.update();
                    }
                }

                double currentTime = Timer.getFPGATimestamp();
                if (currentTime <= nextLoop) {
                    try {
                        Thread.sleep((long) ((nextLoop - currentTime) * 1000));
                    } catch (InterruptedException e) {
                        System.out.println(e.toString());
                        // idk what to do here
                    }
                }
            }
        }
    }
}
