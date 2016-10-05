package com.group.robot;


import lejos.hardware.motor.UnregulatedMotor;
import lejos.robotics.EncoderMotor;
import lejos.robotics.SampleProvider;
import lejos.hardware.Key;
import lejos.hardware.KeyListener;


import lejos.hardware.Button;
import lejos.hardware.Sound;
import lejos.hardware.ev3.LocalEV3;

import lejos.hardware.port.MotorPort;
import lejos.hardware.port.Port;
import lejos.hardware.sensor.EV3GyroSensor;
import lejos.hardware.sensor.SensorModes;


public class RobotTest {

	Port port;
	SensorModes sensor;
	SampleProvider anglePayload;
	float[] sample;

	private static final float KP = 30f;
	private static final float KI = 27.5f;
	private static final float KD = 0;
	private EncoderMotor emMotorA;
	private EncoderMotor emMotorB;
	private float angle;
	private int totalPower;
	private float controllerOutput;
	private float currentError;
	private int diffError = 0;
	private float P;
	private float pPower;
	private long currentTime;
	private long lastTime;
	private long deltaTime;
	private float accumulatedError = 0;
	private float iPower;
	private long timeLimit;
	private EV3GyroSensor ev3Gyro;
	private float referenceAngle;
	private static final float angleIncrement = 1f;

	public RobotTest() {
		buttonSetup();
		setup();
	}

	private void buttonSetup() {
		Button.ESCAPE.addKeyListener(new KeyListener() {
			@Override
			public void keyPressed(Key k) {
			}

			@Override
			public void keyReleased(Key k) {
				System.exit(1);
			}
		});
		Button.LEFT.addKeyListener(new KeyListener() {
			@Override
			public void keyPressed(Key k) {
			}

			@Override
			public void keyReleased(Key k) {
				referenceAngle -= angleIncrement;
				System.out.println("Balance point at: " + referenceAngle);
			}
		});
		Button.RIGHT.addKeyListener(new KeyListener() {
			@Override
			public void keyPressed(Key k) {
			}

			@Override
			public void keyReleased(Key k) {
				referenceAngle += angleIncrement;
				System.out.println("Balance point at: " + referenceAngle);
			}
		});	
	}

	private void setup() {
		try {

			emMotorA = new UnregulatedMotor(MotorPort.A);
			emMotorB = new UnregulatedMotor(MotorPort.B);

			port = LocalEV3.get().getPort("S1");
			ev3Gyro = new EV3GyroSensor(port);
			ev3Gyro.reset();
			sensor = ev3Gyro;
			((EV3GyroSensor) sensor).reset();
			anglePayload = sensor.getMode("Angle");
			sample = new float[anglePayload.sampleSize()];

		} catch (Exception e) {
			e.printStackTrace();
		}
	}

	public void sensorScan() {
		try {
			/*
			 * manually calibrate because of crappy reset function that's not
			 * reliable: !!!If you remove this you will be surprised at how
			 * inaccurate the gyro sensor is!!!
			 */
			Sound.beep();
			System.out.println("Please balance robot and press enter at balance point");
			Button.ENTER.waitForPressAndRelease();
			anglePayload.fetchSample(sample, 0);
			// set the target reference angle (in our world it's around 90
			// degrees)
			referenceAngle = sample[0];
			// show the user how inconsistent the gyro angle is each time...the
			// same angle can range 75->95
			System.out.println("Balance point at: " + referenceAngle);
			Sound.beep();

			// initialize last time for integral part and length to run the
			// program
			lastTime = System.currentTimeMillis();
			while (true) {
				// fetch a sample
				anglePayload.fetchSample(sample, 0);
				angle = sample[0];
				// power from pid output
				totalPower = (int) crop100(PID(angle));

				emMotorA.setPower(totalPower);
				emMotorB.setPower(totalPower);

				// not literally forward, direction already established by PID
				// with + or -
				emMotorA.forward();
				emMotorA.forward();
			}

		} catch (Exception e) {
			e.printStackTrace();
		}
	}

	float PID(float angleInput) {
		controllerOutput = 0;
		// increasing angle makes robot lean back
		// decreasing angle makes robot lean forward
		currentError = (referenceAngle - angle);
		controllerOutput += P(angleInput, currentError);
		controllerOutput += I(angleInput, currentError);
		controllerOutput += D(angleInput);
		return controllerOutput;
	}

	float P(float angleInput, float currentErrorP) {
		pPower = currentErrorP * KP;
		// crop values over 100% because power range for motors is [-100, 100]
		return crop100(pPower);
	}

	float I(float angleInput, float currentErrorI) {
		//some annoying drift accumulates here after a while, causes the robot to lean back more and more
		accumulatedError += currentErrorI * dt();
		//current kludge to get minimize drift by occasionally pulling values down to zero
		if(currentErrorI < 0.5 && accumulatedError < 0.5 ){
			accumulatedError = 0;
		}
		iPower = accumulatedError * KI;
		return crop100(iPower);
	}

	// returns 0 not implemented yet
	float D(float angleInput) {
		return diffError * KD;
	}

	float dt() {
		currentTime = System.currentTimeMillis();
		deltaTime = currentTime - lastTime;
		lastTime = currentTime;
		// convert to seconds
		return deltaTime / 1000f;
	}

	// crop values over 100% because power range for motors is [-100, 100]
	float crop100(float power) {
		if (power > 100) {
			power = 100;
		} else if (power < -100) {
			power = -100;
		}
		return power;
	}

}