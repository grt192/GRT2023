package frc.robot.jetson;

import java.util.ArrayDeque;
import java.util.Queue;

import com.google.gson.Gson;

import org.zeromq.SocketType;
import org.zeromq.ZContext;
import org.zeromq.ZMQ;
import org.zeromq.ZMQ.Poller;
import org.zeromq.ZMQ.Socket;

/**
 * A ZeroMQ socket connection to the jetson. The connection binds to a static port on the RIO, sends data to the 
 * jetson via REP, and receives JSON data via REQ.
 */
public class JetsonConnection extends Thread {
    private final Queue<JetsonConfig> messageQueue = new ArrayDeque<>();

    private final Gson gson = new Gson();

    private static final boolean LOCAL_DEBUG = false;
    private static final String SERVER_IP = LOCAL_DEBUG 
        ? "tcp://*:5800"
        : "tcp://10.1.92.94:5800";
    private static final String RIO_IDENT = "RIO";

    @Override
    public void run() {
        try (ZContext context = new ZContext()) {
            // ZeroMQ DEALER connecting to jetson static IP; jetson sends data via ROUTER.
            System.out.println("Connecting a DEALER to " + SERVER_IP);
            Socket socket = context.createSocket(SocketType.DEALER);
            socket.connect(SERVER_IP);
            socket.setIdentity(RIO_IDENT.getBytes(ZMQ.CHARSET));

            Poller poller = context.createPoller(1);
            poller.register(socket, Poller.POLLIN);

            socket.send("Hello!");

            while (!Thread.currentThread().isInterrupted()) {
                // Poll for inbound messages
                if (poller.poll(0) > 0) {
                    String message = socket.recvStr(0);
                    System.out.println("Received: " + message);

                    JetsonData data = gson.fromJson(message, JetsonData.class);
                    System.out.println(data.x + " " + data.y + " " + data.z + " " + data.ts);

                    long delay = System.currentTimeMillis() - data.ts;
                    System.out.println(delay);
                }

                // If there's a config update, send it to the jetson
                JetsonConfig configUpdate = messageQueue.poll();
                if (configUpdate != null) {
                    String message = gson.toJson(configUpdate);
                    System.out.println("Sending message " + message);
                    socket.send(message);
                }
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
