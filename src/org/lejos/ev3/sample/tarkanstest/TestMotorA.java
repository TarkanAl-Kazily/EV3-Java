/*
 * This program operates a large EV3 motor when a touch sensor is pressed.
 * Make sure that the motor is connected to motor port A and that the
 * touch sensor is connected to sensor port 1.
 */

package org.lejos.ev3.sample.tarkanstest;

import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.port.AnalogPort;
import lejos.hardware.port.MotorPort;
import lejos.hardware.port.Port;
import lejos.hardware.sensor.EV3TouchSensor;
import lejos.hardware.sensor.SensorModes;
import lejos.robotics.RegulatedMotor;
import lejos.robotics.SampleProvider;
import lejos.utility.Delay;

public class TestMotorA {
	static SensorModes buttonSensor;
	static SampleProvider pressed;
	static float[] sample;
	
	public static void main(String[] args) {
		buttonSensor = new EV3TouchSensor(LocalEV3.get().getPort("S1"));
		pressed = buttonSensor.getMode("Touch");
		sample = new float[pressed.sampleSize()];
		RegulatedMotor m = new EV3LargeRegulatedMotor(MotorPort.A);
		while(true) {
			if (buttonPressed()) {
				m.forward();
			} else {
				m.stop();
			}
			Delay.msDelay(100);
		}
	}
	
	public static boolean buttonPressed() {
		if (pressed != null) {
			pressed.fetchSample(sample, 0);
		}
		return (sample[0] == 1);
	}
}
