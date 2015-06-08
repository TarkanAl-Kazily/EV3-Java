/*
 * This program uses an ultrasonic sensor pointing forward (the side with the two
 * tetrix motors) to move forward until it is 30 cm away from an object, then
 * back up until it is 70 cm away. Make sure that the ultrasonic sensor is connected
 * to sensor port 2, and that the motor controller is connected to sensor port 1.
 */

package org.lejos.ev3.sample.tarkanstest;

import lejos.hardware.BrickFinder;
import lejos.hardware.device.tetrix.TetrixControllerFactory;
import lejos.hardware.device.tetrix.TetrixMotorController;
import lejos.hardware.device.tetrix.TetrixServo;
import lejos.hardware.device.tetrix.TetrixServoController;
import lejos.hardware.lcd.Font;
import lejos.hardware.lcd.GraphicsLCD;
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
	static TetrixServoController servoController;
	static TetrixControllerFactory factory;
	static final boolean LEFT = true;
	static final boolean RIGHT = false;
	static SensorModes buttonSensor;
	static SampleProvider pressed;
	
	static SensorModes ultrasonicSensor;
	static SampleProvider distance;
	
	static float[] sample;
	
	static final int MICROSECLOW_DEFAULT = 750;
	static final int MICROSECHIGH_DEFAULT = 2250;
	static final int TRAVELRANGE_DEFAULT = 200;
	
	public static void main(String[] args) {
		initializeControllers();
		initializeMotors();
		initializeUltrasonicSensor();
		RegulatedMotor m = new EV3LargeRegulatedMotor(MotorPort.A);
		setPower(50);
		moveForward();
		m.forward();
		while (distanceRead() >= .3) {
			Delay.msDelay(5);
		}
		stop();
		Delay.msDelay(100);
		moveBackward();
		m.backward();
		while (distanceRead() <=.7) {
			Delay.msDelay(5);
		}
		m.stop();
		stop();
	}
	public static void turn(boolean direction){
		if(LEFT==direction){
			motor1.forward();
			motor2.forward();
		}else{
			motor1.backward();
			motor2.backward();
		}
	}
	/**
	 * Always run this first, before initializing anything else
	 */
	public static void initializeControllers() {
		factory = new TetrixControllerFactory(SensorPort.S1);
		motorController = factory.newMotorController();
		servoController = factory.newServoController();
	}
	
	public static void moveForward() {
		motor1.backward();
		motor2.forward();
	}
	
	public static void moveBackward() {
		motor1.forward();
		motor2.backward();
	}
	
	/**
	 * Run this before using the tetrix motors
	 */
	public static void initializeMotors() {
		if (motorController != null) {
			motor1 = motorController.getBasicMotor(TetrixMotorController.MOTOR_1);
			motor2 = motorController.getBasicMotor(TetrixMotorController.MOTOR_2);
		}
	}
	/**
	 * Run this to get a new TetrixServo object
	 * @param servoNumber the port the servo is connected to
	 * @return the servo object
	 */
	public static TetrixServo initializeServo(int servoNumber) {
		if (servoController != null) {
			return servoController.getServo(servoNumber);
		} else {
			return null;
		}
	}
	
	public static void servoDefaults(TetrixServo servo) {
		servo.setRange(MICROSECLOW_DEFAULT, MICROSECHIGH_DEFAULT, TRAVELRANGE_DEFAULT);
	}
	
	public static void setPower(int power) {
		motor1.setPower(power);
		motor2.setPower(power);
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
	
	public static void initializeUltrasonicSensor() {
		ultrasonicSensor = new EV3UltrasonicSensor(ULTRASONIC_SENSOR_PORT);
		distance = ultrasonicSensor.getMode("Distance");
		sample = new float[distance.sampleSize()];
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
