package org.usfirst.frc.team20.robot;

import java.io.*;
import java.net.*;

public class Util {
	
	
	// readSockets reads in a string called sentence and returns a string from a sockect
	public String readSocket(String ipAddress, int Port, String sentence ) throws IOException
	{
				int timeOut =1500;
		        String socketString = "-1";
			//	BufferedReader inFromUser =
		    //      new BufferedReader(new InputStreamReader(System.in));
		          DatagramSocket clientSocket;
				try {
						clientSocket = new DatagramSocket();
					  //  clientSocket.setSoTimeout(timeOut);
						InetAddress IPAddress = InetAddress.getByName(ipAddress);
						byte[] sendData = new byte[1024];
						byte[] receiveData = new byte[1024];
						//String sentence = inFromUser.readLine();
						System.out.println("sentence = " + sentence);
						sendData = sentence.getBytes();
						DatagramPacket sendPacket = new DatagramPacket(sendData, sendData.length, IPAddress, Port);
					
						clientSocket.send(sendPacket);
						DatagramPacket receivePacket = new DatagramPacket(receiveData, receiveData.length);
						clientSocket.receive(receivePacket);
						String modifiedSentence = new String(receivePacket.getData());
						System.out.println("FROM SERVER:" + modifiedSentence);
						socketString = modifiedSentence;
						clientSocket.close();
						System.out.println("done");
					 } catch (SocketException e) {
					// TODO Auto-generated catch block
					System.out.println("Socket error: " + e.toString());
				}
			
				return socketString;
	}

}
