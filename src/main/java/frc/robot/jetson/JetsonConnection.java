package frc.robot.jetson;

import com.google.gson.Gson;

import org.zeromq.SocketType;
import org.zeromq.ZContext;
import org.zeromq.ZMQ;
import org.zeromq.ZMQ.Socket;

/**
 * A thread maintaining a ZeroMQ PUB/SUB connection to the jetson.
 */
public class JetsonConnection extends Thread {
    private final Gson gson = new Gson();

    private static final boolean LOCAL_DEBUG = true;
    private static final String SERVER_IP = LOCAL_DEBUG 
        ? "tcp://*:5800"
        : "tcp://10.1.92.94:5800";

    @Override
    public void run() {
        try (ZContext context = new ZContext()) {
            // ZeroMQ SUB client connecting to jetson static IP; jetson sends data via PUB.
            System.out.println("Connecting a SUB client to " + SERVER_IP);
            Socket socket = context.createSocket(SocketType.SUB);
            socket.connect(SERVER_IP);
            socket.subscribe("");

            // While the thread is running, keep waiting for messages from the jetson.
            while (!Thread.currentThread().isInterrupted()) {
                String message = socket.recvStr(0);
                System.out.println("Received: " + message);

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
