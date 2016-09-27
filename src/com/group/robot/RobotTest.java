package com.group.robot;

import java.io.IOException;
import java.rmi.RemoteException;
import java.rmi.RemoteException;
import lejos.robotics.SampleProvider;
import lejos.hardware.BrickFinder;
import lejos.hardware.BrickInfo;
import lejos.hardware.Sound;
import lejos.hardware.lcd.GraphicsLCD;
import lejos.hardware.port.Port;
import lejos.hardware.sensor.EV3GyroSensor;
import lejos.remote.ev3.RMIRegulatedMotor;
import lejos.remote.ev3.RMISampleProvider;
import lejos.remote.ev3.RemoteEV3;
import lejos.utility.Delay;

public class RobotTest {

	RemoteEV3 ev3 = null;
	//RMIRegulatedMotor motorA = null;
	//RMIRegulatedMotor motorB = null;
	//RMIRegulatedMotor motorC = null;
	//RMIRegulatedMotor motorD = null;
	RMISampleProvider angleProvider = null;
	RMISampleProvider rateProvider = null;
	RMISampleProvider rateAngleProvider = null;

	public RobotTest() {
		setup();
	}

	private void setup() {
		try {
			BrickInfo[] bricks = BrickFinder.discover();
			for (BrickInfo info : bricks) {
				System.out.println("Ev3 found on Bluetooth ip: " + info.getIPAddress());
			}
			//null if not connected via bluetooth
			String connectedBrick = bricks[0].getIPAddress();
			ev3 = new RemoteEV3(connectedBrick);

		} catch (Exception e) {
			e.printStackTrace();
		}
		ev3.setDefault();
		GraphicsLCD display = ev3.getGraphicsLCD();
		display.clear();
		Sound.beep();

		//motorA = ev3.createRegulatedMotor("A", 'L');
		//motorB = ev3.createRegulatedMotor("B", 'L');


		Sound.beep();
	}

	public boolean sensorScan() throws RemoteException {

		try {

			//angleProvider = ev3.createSampleProvider("S1", "lejos.hardware.sensor.EV3GyroSensor","Angle");
			//rateProvider = ev3.createSampleProvider("S1", "lejos.hardware.sensor.EV3GyroSensor","Rate");
			rateAngleProvider = ev3.createSampleProvider("S1", "lejos.hardware.sensor.EV3GyroSensor","Angle and Rate");

			//motorA.forward();
			//motorB.forward();
			//motorA.stop(false);
			//motorB.stop(false);
			for (int z = 0; z < 200; z++) {
				float[] angleRatePayload = rateAngleProvider.fetchSample();
					System.out.print(angleRatePayload[0] + " ");
					System.out.println(angleRatePayload[1]);
			}

		} catch (Exception e) {
			e.printStackTrace();
		} finally {
			//rateProvider.close();
			//angleProvider.close();
			rateAngleProvider.close();
		}
		return false;

	}

}