package com.group.robot;

import java.io.IOException;
import java.rmi.RemoteException;
import java.rmi.RemoteException;
import lejos.robotics.SampleProvider;
import lejos.hardware.BrickFinder;
import lejos.hardware.BrickInfo;
import lejos.hardware.Sound;
import lejos.hardware.lcd.GraphicsLCD;
import lejos.hardware.motor.Motor;
import lejos.hardware.port.Port;
import lejos.hardware.sensor.EV3GyroSensor;
import lejos.remote.ev3.RMIRegulatedMotor;
import lejos.remote.ev3.RMISampleProvider;
import lejos.remote.ev3.RemoteEV3;
import lejos.utility.Delay;

public class RobotTest {

	RemoteEV3 ev3 = null;
	RMIRegulatedMotor motorA = null;
	RMIRegulatedMotor motorB = null;
	//RMIRegulatedMotor motorC = null;
	//RMIRegulatedMotor motorD = null;
	RMISampleProvider angleProvider = null;
	RMISampleProvider rateProvider = null;
	RMISampleProvider rateAngleProvider = null;
	private float angle;
	private float rate;
	private int angleInt;
	private int reactionAngle;

	public RobotTest() {
		setup();
	}

	private void setup() {
		try {
			BrickInfo[] bricks = BrickFinder.discover();

			//0 if not connected via bluetooth
			if(bricks.length > 0){
				for (BrickInfo info : bricks) {
					System.out.println("Ev3 found on Bluetooth ip: " + info.getIPAddress());
				}
				
				String connectedBrick = bricks[0].getIPAddress();
				ev3 = new RemoteEV3(connectedBrick);
				
				ev3.setDefault();
				GraphicsLCD display = ev3.getGraphicsLCD();
				display.clear();
				Sound.beep();

				
				motorA = ev3.createRegulatedMotor("A", 'L');
				motorB = ev3.createRegulatedMotor("B", 'L');


				Sound.beep();
			}else{
				System.out.println("no brick found on bluetooth");
				throw new Exception("NO BRICK FOUND ON BLUETOOTH");
			}

		} catch (Exception e) {
			e.printStackTrace();
		}

	}

	/**
	 * @return
	 * @throws RemoteException
	 */
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
				angle = angleRatePayload[0];
				rate = angleRatePayload[1];
					//System.out.print(angle + " ");
					System.out.println(rate);
					angleInt = (int) (angle - 93);
					//System.out.print(angleInt + " ");
	

					//squaring makes higer values increase rapidly for 
					//dividing slows down motor speed
					//motorA.setSpeed((int)Math.abs(rate * rate / 2));	
					//motorB.setSpeed((int)Math.abs(rate * rate / 2));	
					motorA.setSpeed((int)Math.abs(rate * (rate*.08) * 40));	
					motorB.setSpeed((int)Math.abs(rate * (rate*.08) * 40));
					
					//scale down the higher degrees and increase the lower ones
					if(angleInt < 0){
						//this lets sqrt work with negative numbers
						//reactionAngle = (int)(Math.sqrt(Math.abs(angleInt)) * 5);
						reactionAngle = -angleInt;

						//reactionAngle = -(angleInt * angleInt * angleInt);
						//motorA.setSpeed((int)Math.abs(rate));
						motorA.forward();
						motorB.forward();
						//motorB.setSpeed((int)Math.abs(rate));


					} else {
						motorA.backward();
						motorB.backward();
						//reactionAngle = -(int)(Math.sqrt(angleInt) * 5);
						reactionAngle = -angleInt;
						//reactionAngle = -(angleInt * angleInt * angleInt);
						//motorA.setSpeed(100);		
						//motorB.setSpeed(100);		

					}
					//System.out.println(reactionAngle);
					//any motor methods may interrupt rotate
					reactionAngle = -angleInt;

					
					//motorA.rotate(reactionAngle, true);
					//motorB.rotate(reactionAngle, true);
			}

		} catch (Exception e) {
			e.printStackTrace();
		} finally {
			//rateProvider.close();
			//angleProvider.close();
			motorA.stop(false);
			motorA.stop(false);
			motorA.close();
			motorB.close();
			rateAngleProvider.close();
			ev3 = null;
		}
		return false;

	}

}