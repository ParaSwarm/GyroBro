package com.group.robot;

import lejos.hardware.motor.UnregulatedMotor;
import lejos.hardware.ev3.LocalEV3;
import lejos.robotics.EncoderMotor;
import lejos.robotics.SampleProvider;
import lejos.robotics.filter.IntegrationFilter;
import lejos.robotics.filter.LowPassFilter;
import lejos.robotics.filter.OffsetCorrectionFilter;
import lejos.hardware.Key;
import lejos.hardware.KeyListener;

import lejos.hardware.Button;
import lejos.hardware.Sound;
import lejos.hardware.ev3.LocalEV3;

import lejos.hardware.port.MotorPort;
import lejos.hardware.port.Port;
import lejos.hardware.sensor.EV3GyroSensor;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.hardware.sensor.HiTechnicGyro;
import lejos.hardware.sensor.SensorMode;
import lejos.hardware.sensor.SensorModes;

public class RobotTest {

	Port port;
	SensorModes sensor;
	SampleProvider ratePayload;
	private static final float batteryVoltage = LocalEV3.get().getPower().getVoltage();
	private static final float bGain = 13f / batteryVoltage;
	float[] ultraSample;

	private static final float KP = 30f * bGain;
	// private static final float KI = 85f;
	private static final float KI = 100f * bGain;
	// private static final float KD = 0.05f;
	private static final float KD = 0.05f * bGain;
	private EncoderMotor emMotorA;
	private EncoderMotor emMotorB;
	private float angle;
	private int totalPower;
	private float controllerOutput;
	private float currentError;
	private float diffError = 0;
	private float P;
	private float pPower;
	private long currentTimeDt;
	private long lastTime;
	private long deltaTime;
	private float accumulatedError = 0;
	private float iPower;
	private long timeLimit;
	// private HiTechnicGyro hiTechGyro;
	private float referenceAngle;
	// 0.00006475 is too weak
	// .000065 too much
	private static final float biasIncrement = 0.000065f;
	private float previousError;
	private float dt;
	private long currentTime;
	private int nSamples = 1;
	private float mean;
	private float delta;
	private float M2;
	private float highAngle;
	private float lowAngle;
	private float stdDeviationHigh = 0;
	private float stdDeviation;
	private int counter;
	private boolean stdHigh;
	private float stdDeviationLow = 0;
	private boolean stdLow;
	private float referenceAngleHigh;
	private float referenceAngleLow;
	private float referenceAngleTemp = 0;
	// private IntegrationFilter integratedPayload;
	// private OffsetCorrectionFilter offsetFilter;
	private float currentAngle = 0;
	private EV3UltrasonicSensor EV3Ultra;
	private SampleProvider ultraSonicPayload;
	private Port port2;
	private HiTechnicGyro hiTechGyro;
	private SampleProvider gyroPayload;
	private SampleProvider anglePayload;
	private SampleProvider offsetPayload;
	private float[] gyroSample;
	private float distance;
	private float avgDistance;
	private int distanceCounter;
	private static final float angleIncrement = 0.04f;

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
				referenceAngle -= angleIncrement * 2;
			}
		});
		Button.RIGHT.addKeyListener(new KeyListener() {
			@Override
			public void keyPressed(Key k) {
			}

			@Override
			public void keyReleased(Key k) {
				referenceAngle += angleIncrement * 2;
			}
		});
	}

	private void setup() {
		try {

			emMotorA = new UnregulatedMotor(MotorPort.A);
			emMotorB = new UnregulatedMotor(MotorPort.B);

			port = LocalEV3.get().getPort("S1");
			port2 = LocalEV3.get().getPort("S2");
			EV3Ultra = new EV3UltrasonicSensor(port);
			hiTechGyro = new HiTechnicGyro(port2);
			// ev3Gyro.reset();
			// sensor = hiTechGyro;
			// ((EV3GyroSensor) sensor).reset();

			// anglePayload = hiTechGyro.getMode("Rate");
			ultraSonicPayload = EV3Ultra.getDistanceMode();
			gyroPayload = hiTechGyro.getRateMode();
			anglePayload = new IntegrationFilter(gyroPayload);
			offsetPayload = new OffsetCorrectionFilter(anglePayload);

			// lowPassPayload = new LowPassFilter(ratePayload, 10f);
			// sample = new float[integratedPayload.sampleSize()];
			ultraSample = new float[ultraSonicPayload.sampleSize()];
			gyroSample = new float[offsetPayload.sampleSize()];
			System.out.println("Sample size is :" + ultraSonicPayload.sampleSize());

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
			System.out.println(batteryVoltage);
			Button.ENTER.waitForPressAndRelease();
			ultraSonicPayload.fetchSample(ultraSample, 0);
			offsetPayload.fetchSample(gyroSample, 0);
			// set the target reference angle (in our world it's around 90
			// degrees)
			System.out.println("Angle is: " + gyroSample[0]);
			referenceAngle = gyroSample[0];
			Thread.sleep(2000);
			//System.out.println(referenceAngle)


			// show the user how inconsistent the gyro angle is each time...the
			// same angle can range 75->95
			System.out.println("Balance point at: " + referenceAngle);
			Sound.beep();

			// initialize last time for integral part and length to run the
			// program
			lastTime = System.currentTimeMillis();
			while (true) {
				// if((System.currentTimeMillis()-lastTime % 1000)){
				//
				// }
				// set dt
				dt = dt();
				Thread.sleep(1);
				// fetch a sample
				ultraSonicPayload.fetchSample(ultraSample, 0);
				offsetPayload.fetchSample(gyroSample, 0);
				angle = gyroSample[0];
				distance = ultraSample[0];
				System.out.println(ultraSample[0]);
				//angle = getIntegratedAngle(sample[0]);
				avgDistance += (distance / ++distanceCounter);
				if(avgDistance > 0.025f && avgDistance < 0.060f){
					avgDistance = 0;
					//too high
					if(avgDistance > 0.055f){
						referenceAngle += angleIncrement;
					//too low
					}else if(avgDistance < 0.045f){
						referenceAngle -= angleIncrement;
					}
				}

				// test slightly higher angle or lower angle for comparing
				// deviation

				// power from pid output

				//deviationCalibration(angle); // stdDevation set

				totalPower = (int) crop100(PID(angle));

				emMotorA.setPower(totalPower);
				emMotorB.setPower(totalPower);

				// not literally forward, direction already established by PID
				// with + or -

			}

		} catch (Exception e) {
			e.printStackTrace();
		} finally {
			EV3Ultra.close();
		}
	}

	void deviationCalibration(float angle) {

		if (counter % 100 != 0) {
			counter++;
		} else {
			counter = 1;
			if (stdHigh) {

				stdHigh = false;
				stdLow = true;
				if (referenceAngleTemp != 0) {
					referenceAngle = referenceAngleTemp;
				}
				referenceAngleLow = referenceAngle - angleIncrement;
				referenceAngleTemp = referenceAngle;
				referenceAngle = referenceAngleLow;
			} else {
				stdLow = false;
				stdHigh = true;
				if (referenceAngleTemp != 0) {
					referenceAngle = referenceAngleTemp;
				}
				referenceAngleHigh = referenceAngle + angleIncrement;
				referenceAngleTemp = referenceAngle;
				referenceAngle = referenceAngleHigh;
			}

		}
		if (nSamples == 100) {
			if (stdHigh) {

				stdDeviationHigh = M2 / (nSamples - 1);
				// System.out.println("High" + stdDeviationHigh);
			} else {
				stdDeviationLow = M2 / (nSamples - 1);
				// System.out.println("Low " + stdDeviationLow);
			}
			nSamples = 1;
			mean = 0.0f;
			M2 = 0.0f;

		} else {
			nSamples += 1;
			delta = angle - mean;
			mean += delta / nSamples;
			M2 += delta * (angle - mean);
		}
		if ((stdDeviationHigh > 0.5 || stdDeviationLow > 0.5) && Math.abs(stdDeviationHigh - stdDeviationLow) > 0.5) {
			if (stdDeviationHigh < stdDeviationLow) {
				referenceAngle = referenceAngleHigh;
			} else {
				referenceAngle = referenceAngleLow;
			}
		} else {
			if (referenceAngleTemp != 0) {
				referenceAngle = referenceAngleTemp;
			}
		}
	}

	float PID(float angleInput) {
		controllerOutput = 0;
		diffError = 0;
		// increasing angle makes robot lean back
		// decreasing angle makes robot lean forward
		// calculate new reference angle using lowest standard deviation every
		// 100 iterations

		//

		currentError = (referenceAngle - angle);

		controllerOutput += P(angleInput, currentError);
		controllerOutput += I(angleInput, currentError);
		controllerOutput += D();
		return controllerOutput;
	}

	float getIntegratedAngle(float accelerationInput) {
		// 418... is the resting acceleration for the hi technic gyro
		currentAngle += (accelerationInput + 418.25354f) * dt;
		return currentAngle;
	}

	float P(float angleInput, float currentErrorP) {
		pPower = currentErrorP * KP;
		// crop values over 100% because power range for motors is [-100, 100]
		return crop100(pPower);
	}

	float I(float angleInput, float currentErrorI) {
		// some annoying drift accumulates here after a while, causes the robot
		// to lean back more and more
		accumulatedError += currentErrorI * dt;
		// current kludge to get minimize drift by occasionally pulling values
		// down to zero
		if (currentErrorI < 0.5 && accumulatedError < 0.5) {
			accumulatedError = 0;
		}
		iPower = accumulatedError * KI;
		return crop100(iPower);
	}

	// returns 0 not implemented yet
	float D() {
		diffError = (currentError - previousError) / dt;
		previousError = currentError;
		return (diffError * KD);
	}

	float dt() {
		currentTimeDt = System.currentTimeMillis();
		deltaTime = currentTimeDt - lastTime;
		lastTime = currentTimeDt;
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