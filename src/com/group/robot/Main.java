package com.group.robot;

import java.rmi.RemoteException;

public class Main {

	public static void main(String[] args) {
		// TODO Auto-generated method stub
		RobotTest robotTest = new RobotTest();
		try {
			robotTest.sensorScan();
		} catch (RemoteException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		} catch (Exception e){
			e.printStackTrace();
		}
	}

}
