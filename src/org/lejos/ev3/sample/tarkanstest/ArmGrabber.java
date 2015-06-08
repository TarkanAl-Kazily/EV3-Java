/*
 * Using the arm, this program moves the robot forward a short distance,
 * grabs an object, and lifts it. Make sure that the grabbing servo is 
 * connected to servo channel 2, and that the lifting servo is connected to
 * channel 1. The controllers should be connected to each other to sensor port
 * 1.
 */

package org.lejos.ev3.sample.tarkanstest;

import lejos.hardware.device.tetrix.TetrixControllerFactory;
import lejos.hardware.device.tetrix.TetrixMotorController;
import lejos.hardware.device.tetrix.TetrixServo;
import lejos.hardware.device.tetrix.TetrixServoController;
import lejos.hardware.port.Port;
import lejos.hardware.port.SensorPort;
import lejos.hardware.sensor.SensorModes;
import lejos.robotics.DCMotor;
import lejos.robotics.SampleProvider;
import lejos.robotics.Servo;
import lejos.utility.Delay;

public class ArmGrabber {
	static final Port LIGHT_SENSOR_PORT = SensorPort.S3;
	static final Port ULTRASONIC_SENSOR_PORT = SensorPort.S2;
	static final Port TOUCH_SENSOR_PORT = SensorPort.S4;
	static DCMotor motor1;
	static DCMotor motor2;
	static TetrixMotorController motorController;
	static TetrixServoController servoController;
	static TetrixControllerFactory factory;
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
		Servo lift = initializeServo(0); //Up is 0, down is 200
		Servo grab = initializeServo(1); //Closed is 0, open is 200
		setPower(40);
		grab.setAngle(200);
		lift.setAngle(200);
		Delay.msDelay(1000);
		moveForward();
		Delay.msDelay(400);
		stop();
		grab.setAngle(50);
		Delay.msDelay(500);
		lift.setAngle(0);
		Delay.msDelay(1000);
	}
	
	public static void moveForward() {
		motor1.backward();
		motor2.forward();
	}
	
	public static void stop() {
		motor1.stop();
		motor2.stop();
	}
	
	public static void setPower(int power) {
		motor1.setPower(power);
		motor2.setPower(power);
	}
	
	/**
	 * Always run this first, before initializing anything else
	 */
	public static void initializeControllers() {
		factory = new TetrixControllerFactory(SensorPort.S1);
		motorController = factory.newMotorController();
		servoController = factory.newServoController();
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
	
	public static void servoDefaults(Servo servo) {
		servo.setRange(MICROSECLOW_DEFAULT, MICROSECHIGH_DEFAULT, TRAVELRANGE_DEFAULT);
	}
}
