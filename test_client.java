import java.io.*;
import java.net.*;

public class test_client{

	public static void main(String[] a){
		test_client c = new test_client("172.217.31.206", 443);
		//c.stopSignal();
	}

	Socket socket;
	DataOutputStream out;
	DataInputStream in;

	public test_client(String ip, int port){
		try{
			this.socket = new Socket(ip, port);
			this.in = new DataInputStream(System.in);
			this.out = new DataOutputStream(this.socket.getOutputStream());
		}catch (UnknownHostException e){
			System.out.println(e);
			return ;
		}catch (IOException e){
			System.out.println(e);
			return ;
		}
		System.out.println("connected");
	}

	public void stopSignal(){
		for (int t = 0; t < 1000; ++t){
            try{
                String signal = "STOP";
                this.out.writeUTF(signal);
            }catch (IOException e){

            }
        }

        try{
            this.out.close();
            this.socket.close();
        }catch (IOException e){

        }
    }
}


