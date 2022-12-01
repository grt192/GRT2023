package frc.robot.jetson;

import com.google.gson.Gson;

import org.zeromq.SocketType;
import org.zeromq.ZContext;
import org.zeromq.ZMQ;

/**
 * A ZeroMQ socket connection to the jetson. The connection binds to a static port on the RIO, sends data to the 
 * jetson via REP, and receives JSON data via REQ.
 */
public class JetsonConnection extends Thread {
    private final Gson gson = new Gson();

    private static final String SERVER_IP = "tcp://*:5000";

    @Override
    public void run() {
        try (ZContext context = new ZContext()) {
            // ZeroMQ subscription client connecting to jetson static IP, jetson publishes via PUB.
            System.out.println("Connecting a SUB client to " + SERVER_IP);
            ZMQ.Socket socket = context.createSocket(SocketType.SUB);
            socket.connect(SERVER_IP);
            socket.subscribe("");

            // While the thread is still running, keep waiting for messages from the jetson.
            while (!Thread.currentThread().isInterrupted()) {
                String message = socket.recvStr(0);
                System.out.println(message);

                JetsonData data = gson.fromJson(message, JetsonData.class);
                System.out.println(data.x + " " + data.y + " " + data.z + " " + data.ts);

                long delay = System.currentTimeMillis() - data.ts;
                System.out.println(delay);
            }
        }
    }

    /**
     * Debug main method to test connections locally. Constructs and runs the connection
     * as a thread.
     */
    public static void main(String... args) {
        JetsonConnection connection = new JetsonConnection();
        connection.start();
    }
}
