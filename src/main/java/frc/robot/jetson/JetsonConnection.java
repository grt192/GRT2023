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

    private static final boolean LOCAL_DEBUG = true;
    private static final String SERVER_IP = LOCAL_DEBUG 
        ? "tcp://*:5000"
        : "tcp://10.1.92.94:5000";

    @Override
    public void run() {
        try (ZContext context = new ZContext()) {
            // ZeroMQ DEALER connecting to jetson static IP; jetson sends data via ROUTER.
            System.out.println("Connecting a DEALER to " + SERVER_IP);
            ZMQ.Socket socket = context.createSocket(SocketType.DEALER);
            socket.connect(SERVER_IP);
            socket.setIdentity("RIO".getBytes(ZMQ.CHARSET));

            // While the thread is still running, keep waiting for messages from the jetson.
            while (!Thread.currentThread().isInterrupted()) {
                socket.recv(0);
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
