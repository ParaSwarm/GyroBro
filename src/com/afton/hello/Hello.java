package com.afton.hello;

import java.io.IOException;
import java.rmi.RemoteException;
import java.rmi.RemoteException;
import lejos.robotics.SampleProvider;
import lejos.hardware.BrickFinder;
import lejos.hardware.BrickInfo;
import lejos.hardware.Sound;
import lejos.hardware.lcd.GraphicsLCD;
import lejos.remote.ev3.RMIRegulatedMotor;
import lejos.remote.ev3.RMISampleProvider;
import lejos.remote.ev3.RemoteEV3;
import lejos.utility.Delay;

public class Hello {

	RemoteEV3 ev3 = null;
	RMIRegulatedMotor motorA = null;
	RMIRegulatedMotor motorB = null;
	RMIRegulatedMotor motorC = null;
	RMIRegulatedMotor motorD = null;

	// FrontDorOpen
	private boolean isOpen = false;

	public boolean getIsOpen() {
		return isOpen;
	}

	public Hello() {
		setup();
	}

	private void setup() {
		try {
			BrickInfo[] bricks = BrickFinder.discover();
			for (BrickInfo info : bricks) {
				System.out.println("Ev3 found on Bluetooth ip: " + info.getIPAddress());
			}
			String test = bricks[0].getIPAddress();
			ev3 = new RemoteEV3(bricks[0].getIPAddress());

		} catch (Exception e) {
			e.printStackTrace();
		}

		ev3.setDefault();
		GraphicsLCD display = ev3.getGraphicsLCD();
		display.clear();
		Sound.beep();

		motorA = ev3.createRegulatedMotor("A", 'L');
		motorB = ev3.createRegulatedMotor("B", 'L');
		// motorC = ev3.createRegulatedMotor("C",'L');
		// motorD = ev3.createRegulatedMotor("D",'L');

		Sound.beep();
		try {
			// test the first motor
			motorA.rotate(360);
		} catch (RemoteException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
	}

	public boolean forward(int sek) throws RemoteException {

		RMISampleProvider angleProvider = ev3.createSampleProvider("S1", "lejos.hardware.sensor.EV3GyroSensor",
				"Angle");
		float[] sampleTest = angleProvider.fetchSample();
		for (float i : sampleTest) {
			System.out.print(i);
		}
		motorA.forward();
		motorB.forward();
		Delay.msDelay(sek * 1000);
		motorA.stop(false);
		motorB.stop(false);
		return false;
	}

}