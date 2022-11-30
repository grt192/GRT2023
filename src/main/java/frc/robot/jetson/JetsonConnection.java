package frc.robot.jetson;

import org.zeromq.SocketType;
import org.zeromq.ZContext;
import org.zeromq.ZMQ;

/**
 * A ZeroMQ socket connection to the jetson. Binds to a static port on the RIO, sends data to the jetson 
 * via REP, and receives data via REQ.
 */
public class JetsonConnection extends Thread {
    @Override
    public void run() {
        try (ZContext context = new ZContext()) {
            // ZeroMQ reply server bound to static IP, jetson connects via REQ.
            ZMQ.Socket socket = context.createSocket(SocketType.REP);
            socket.bind("tcp://10.1.92.12:5000");

            // While the thread is still running, keep waiting for messages from the jetson.
            while (!Thread.currentThread().isInterrupted()) {
                String message = socket.recvStr(0);
                System.out.println(message);

                Thread.sleep(1000);

                socket.send("Hello!");
            }
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
    }
}
