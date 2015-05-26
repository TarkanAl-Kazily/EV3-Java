package org.lejos.ev3.sample.tarkanstest;

import lejos.hardware.device.tetrix.TetrixControllerFactory;
import lejos.hardware.device.tetrix.TetrixMotorController;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.port.MotorPort;
import lejos.hardware.port.Port;
import lejos.hardware.port.SensorPort;
import lejos.hardware.sensor.EV3TouchSensor;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.hardware.sensor.SensorModes;
import lejos.robotics.DCMotor;
import lejos.robotics.RegulatedMotor;
import lejos.robotics.SampleProvider;
import lejos.utility.Delay;

public class TetrixGo {

	static final Port LIGHT_SENSOR_PORT = SensorPort.S3;
	static final Port ULTRASONIC_SENSOR_PORT = SensorPort.S2;
	static final Port TOUCH_SENSOR_PORT = SensorPort.S4;
	static DCMotor motor1;
	static DCMotor motor2;
	static TetrixMotorController motorController;
	static TetrixControllerFactory factory;
	
	static SensorModes buttonSensor;
	static SampleProvider pressed;
	
	static SensorModes ultrasonicSensor;
	static SampleProvider distance;
	
	static float[] sample;
	
	
	public static void main(String[] args) {
		whileDistanceAway(.1);
	}
	
	public static void moveForward(int duration) {
		factory = new TetrixControllerFactory(SensorPort.S1);
		motorController = factory.newMotorController();
		motor1 = motorController.getBasicMotor(TetrixMotorController.MOTOR_1);
		motor2 = motorController.getBasicMotor(TetrixMotorController.MOTOR_2);
		setPower(50);
		forward();
		Delay.msDelay(duration);
		stop();
	}
	
	public static void setPower(int power) {
		motor1.setPower(power);
		motor2.setPower(power);
	}
	
	public static void forward() {
		motor1.forward();
		motor2.forward();
	}

	public static void stop() {
		motor1.stop();
		motor2.stop();
	}

	public static void whileButtonPressed() {
		buttonSensor = new EV3TouchSensor(TOUCH_SENSOR_PORT);
		pressed = buttonSensor.getMode("Touch");
		sample = new float[pressed.sampleSize()];
		RegulatedMotor m = new EV3LargeRegulatedMotor(MotorPort.A);
		while (true) {
			if (buttonPressed()) {
				m.forward();
			} else {
				m.stop();
			}
			Delay.msDelay(100);
		}
	}
	
	public static void whileDistanceAway(double dist) {
		ultrasonicSensor = new EV3UltrasonicSensor(ULTRASONIC_SENSOR_PORT);
		distance = ultrasonicSensor.getMode("Distance");
		sample = new float[distance.sampleSize()];
		RegulatedMotor m = new EV3LargeRegulatedMotor(MotorPort.A);
		while (true) {
			if (distanceRead() <= dist) {
				m.forward();
			} else {
				m.stop();
			}
			Delay.msDelay(10);
		}
	}
	
	public static double distanceRead() {
		if (distance != null) {
			distance.fetchSample(sample, 0);
		}
		return sample[0];
	}
	
	public static boolean buttonPressed() {
		if (pressed != null) {
			pressed.fetchSample(sample, 0);
		}
		return (sample[0] == 1);
	}
}
