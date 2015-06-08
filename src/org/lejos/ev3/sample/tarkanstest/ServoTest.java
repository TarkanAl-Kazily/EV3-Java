/*
 * Moves servo 1 to a number of different positions, and prints out its angle
 * each time. Make sure that the servo controller is connected to the motor
 * controller, which needs to be connected to sensor port 1. It would be best
 * for the servo to not be part of any arm.
 */

package org.lejos.ev3.sample.tarkanstest;

import lejos.hardware.BrickFinder;
import lejos.hardware.device.tetrix.TetrixControllerFactory;
import lejos.hardware.device.tetrix.TetrixMotorController;
import lejos.hardware.device.tetrix.TetrixServo;
import lejos.hardware.device.tetrix.TetrixServoController;
import lejos.hardware.lcd.Font;
import lejos.hardware.lcd.GraphicsLCD;
import lejos.hardware.lcd.LCD;
import lejos.hardware.port.Port;
import lejos.hardware.port.SensorPort;
import lejos.hardware.sensor.SensorModes;
import lejos.robotics.DCMotor;
import lejos.robotics.SampleProvider;
import lejos.robotics.Servo;
import lejos.utility.Delay;

public class ServoTest {

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
		Servo servo1 = initializeServo(0);
		//servoDefaults(servo1);
		GraphicsLCD g = BrickFinder.getDefault().getGraphicsLCD();
		final int SW = g.getWidth();
        final int SH = g.getHeight();
		float angle = servo1.getAngle();
        g.setFont(Font.getSmallFont());
		String foo = Float.toString(angle);
		g.drawString(foo, SW/2, SH/2, GraphicsLCD.BASELINE|GraphicsLCD.HCENTER);
		Delay.msDelay(1000);
		servo1.setAngle(20);
		g.clear();
		foo = Float.toString(servo1.getAngle());
		g.drawString(foo, SW/2, SH/2, GraphicsLCD.BASELINE|GraphicsLCD.HCENTER);
		Delay.msDelay(1000);
		servo1.setAngle(100);
		foo = Float.toString(servo1.getAngle());
		g.clear();
		g.drawString(foo, SW/2, SH/2, GraphicsLCD.BASELINE|GraphicsLCD.HCENTER);
		Delay.msDelay(1000);
		servo1.setAngle(180);
		foo = Float.toString(servo1.getAngle());
		g.clear();
		g.drawString(foo, SW/2, SH/2, GraphicsLCD.BASELINE|GraphicsLCD.HCENTER);
		Delay.msDelay(1000);
		/*while (angle < 140) {
			foo = Float.toString(angle);
			servo1.setAngle(angle);
			Delay.msDelay(2);
			g.clear();
			g.drawString(foo, SW/2, SH/2, GraphicsLCD.BASELINE|GraphicsLCD.HCENTER);
		}
		*/
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
