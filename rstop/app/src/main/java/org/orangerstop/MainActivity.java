package org.orangerstop;

import androidx.appcompat.app.AppCompatActivity;
import android.os.Bundle;
import android.view.View;
import android.widget.Button;
import android.widget.TextView;

import java.io.*;
import java.net.*;

public class MainActivity extends AppCompatActivity {

    public final String ip = "192.168.168.32";
    public final int port = 6969;

    public boolean clicked = false;
    @Override
    protected void onCreate(Bundle savedInstanceState) {
        System.out.println("test");
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_main);

        TextView success = findViewById(R.id.success);
        TextView status = findViewById(R.id.status);
        status.setText("Status: Running");

        StopButtonListener listener = new StopButtonListener(this.ip, this.port, status, success, this.clicked);

        Button stop_button = findViewById(R.id.button);
        stop_button.setOnClickListener(listener);
    }
}

class StopButtonListener implements View.OnClickListener{
    String ip;
    int port;
    TextView status;
    TextView success;
    Thread signalSender;
    client c;
    boolean clicked;
    public StopButtonListener(String ip, int port, TextView status, TextView success, boolean clicked){
        this.ip = ip;
        this.port = port;
        this.success = success;
        this.status = status;
        this.c = new client(this.ip, this.port, this.success);
        this.signalSender = new Thread(this.c);
        this.clicked = clicked;
    }

    @Override
    public void onClick(View v){
        if (this.clicked) return;
        this.clicked = true;
        this.status.setText("Status: STOP");
        this.signalSender.start();
        //c.run();
    }
}

class client implements Runnable{
    Socket socket;
    DataOutputStream out;
    String ip;
    int port;
    TextView success;
    public client(String ip, int port, TextView success){
        this.ip = ip;
        this.port = port;
        this.success = success;
    }

    @Override
    public void run(){
        // connecting to the computer
        // each thread generates a fresh socket and kills it after the work is completed

        //generating the socket
        try{
            this.socket = new Socket(this.ip, this.port);
            this.out = new DataOutputStream(this.socket.getOutputStream());
            //this.success.setText("connected to " + this.ip + ":" + this.port);
        }catch (UnknownHostException e){
            //this.success.setText("[socket generation]: " + e.toString());
            return ;
        }catch (IOException e){
            //this.success.setText("[socket generation]: " + e.toString());
            return ;
        }

        //sending the stop signal
        for (int t = 0; t < 1000; ++t){
            try{
                String signal = "STOP";
                this.out.writeUTF(signal);
            }catch (IOException e){
                //this.success.setText("[throwing signal]: " + e.toString());
            }
        }

        //killing the socket object
        try{
            this.out.close();
            this.socket.close();
        }catch (IOException e){
            //this.success.setText("[terminate socket]: " + e.toString());
        }

        //test
        /*for (int t = 0; t < 10; ++t){
            //this.success.setText(t);
            try{
                Thread.sleep(1000);
            }catch (InterruptedException e){

            }
        }

        this.success.setText("yo");*/
    }
}