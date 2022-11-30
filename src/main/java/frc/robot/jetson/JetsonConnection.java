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

    private static final String SERVER_IP = "tcp://10.1.92.12:5000";

    @Override
    public void run() {
        try (ZContext context = new ZContext()) {
            // ZeroMQ reply server bound to static IP, jetson connects via REQ.
            System.out.println("Connecting a REP server to " + SERVER_IP);
            ZMQ.Socket socket = context.createSocket(SocketType.REP);
            socket.bind(SERVER_IP);

            // While the thread is still running, keep waiting for messages from the jetson.
            while (!Thread.currentThread().isInterrupted()) {
                String message = socket.recvStr(0);
                System.out.println(message);

                JetsonData data = gson.fromJson(message, JetsonData.class);
                System.out.println(data.x + " " + data.y + " " + data.z);

                Thread.sleep(1000);

                socket.send("Hello!");
            }
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
    }
}
