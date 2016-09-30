package com.group.robot;

import java.rmi.RemoteException;

public class Main {

	public static void main(String[] args) {
		// TODO Auto-generated method stub
		try {
			RobotTest robotTest = new RobotTest();
			robotTest.sensorScan();
		} catch (Exception e){
			e.printStackTrace();
		}
	}

}
