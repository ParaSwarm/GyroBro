package com.group.robot;

import java.io.IOException;
import java.rmi.RemoteException;
import java.rmi.RemoteException;
import lejos.hardware.motor.UnregulatedMotor;
import lejos.robotics.EncoderMotor;
import lejos.robotics.SampleProvider;

import lejos.robotics.filter.MeanFilter;
import lejos.hardware.BrickFinder;
import lejos.hardware.BrickInfo;
import lejos.hardware.Sound;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.lcd.GraphicsLCD;
import lejos.hardware.motor.Motor;
import lejos.hardware.port.MotorPort;
import lejos.hardware.port.Port;
import lejos.hardware.sensor.EV3GyroSensor;
import lejos.hardware.sensor.SensorModes;
import lejos.remote.ev3.RMIRegulatedMotor;
import lejos.remote.ev3.RMISampleProvider;
import lejos.remote.ev3.RemoteEV3;
import lejos.utility.Delay;

public class RobotTest {

	Port port;
	SensorModes sensor;
	SampleProvider anglePayload;
	float[] sample;
	
	private int angleInt;
	private int reactionAngle;
	private EncoderMotor emMotorA;
	private EncoderMotor emMotorB;
	private float angle;
	private int angleAbs;
	public RobotTest() {
		setup();
	}

	private void setup() {
		try {
				emMotorA = new UnregulatedMotor(MotorPort.A);
				emMotorB = new UnregulatedMotor(MotorPort.B);

				// get a port instance
				port = LocalEV3.get().getPort("S1");
				// Get an instance of the Ultrasonic EV3 sensor
				sensor = new EV3GyroSensor(port);
				// get an instance of this sensor in measurement mode
				anglePayload = sensor.getMode("Angle");
				// initialise an array of floats for fetching samples
				sample = new float[anglePayload.sampleSize()];

				Sound.beep();


		} catch (Exception e) {
			e.printStackTrace();
		}

	}

	/**
	 * @return
	 * @throws RemoteException
	 */
	public boolean sensorScan() {
		try {
			for (int z = 0; z < 1000; z++) {
				// fetch a sample
				anglePayload.fetchSample(sample, 0);
				angle = sample[0];
				System.out.println(angle);
					angleInt = (int) (angle - 90);
					angleAbs = Math.abs(angleInt);
					
					//squaring makes higer values increase rapidly for 
					//dividing slows down motor speed
					//emMotorA.setPower(angleAbs * 100);
					//emMotorB.setPower(* 100);
					if(angleAbs > 20){
						angleAbs = 20;
					}
					
//					emMotorA.setPower((angleAbs / 20) * 100);
//					emMotorB.setPower((angleAbs / 20) * 100);
					//(100 - 50) / (100 - 50) * (angleAbs - 100 + 100
					emMotorA.setPower(angleAbs + 50);
					emMotorB.setPower(angleAbs + 50);

					//scale down the higher degrees and increase the lower ones
					//emMotorA.setSpeed((int)Math.abs(rate * rate / 2));	
					//emMotorB.setSpeed((int)Math.abs(rate * rate / 2));	
					//motorA.setSpeed((int)Math.abs(rate * (rate*.08) * 40));	
					//motorB.setSpeed((int)Math.abs(rate * (rate*.08) * 40));
					
					if(angleInt < 0){
						//less than 90
						emMotorA.forward();
						emMotorB.forward();

					} else {
						//greater than 90
						emMotorA.backward();
						emMotorB.backward();
					}
			}

		} catch (Exception e) {
			e.printStackTrace();
		} finally {

		}
		return false;

	}

}