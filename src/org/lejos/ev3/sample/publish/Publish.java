package org.lejos.ev3.sample.publish;

import java.io.IOException;

import lejos.hardware.Button;
import lejos.hardware.lcd.LCD;
import lejos.hardware.port.SensorPort;
import lejos.hardware.sensor.EV3GyroSensor;
import lejos.hardware.sensor.EV3IRSensor;
import lejos.hardware.sensor.EV3UltrasonicSensor;

import lejos.robotics.SampleProvider;
import lejos.robotics.filter.PublishFilter;
import lejos.utility.Delay;

public class Publish {
	public static void main(String[] args) throws IOException {
		float frequency = 1; // 1 sample per second
		EV3IRSensor irSensor = new EV3IRSensor(SensorPort.S1);
		SampleProvider sp = new PublishFilter(irSensor.getDistanceMode(), "IR range readings", frequency);
		float[] sample = new float[sp.sampleSize()];
		
		EV3UltrasonicSensor sonicSensor = new EV3UltrasonicSensor(SensorPort.S2);
		SampleProvider sp2 = new PublishFilter(sonicSensor.getDistanceMode(), "Ultrasonic readings", frequency);
		float[] sample2 = new float[sp2.sampleSize()];
		
		EV3GyroSensor gyroSensor = new EV3GyroSensor(SensorPort.S3);
		SampleProvider sp3 = new PublishFilter(gyroSensor.getRateMode(), "Gyro readings", frequency);
		float[] sample3 = new float[sp3.sampleSize()];
		
		while(Button.ESCAPE.isUp()) {
			sp.fetchSample(sample, 0);
			LCD.clear(3);
			LCD.drawString("IR: " + sample[0],0,3);
			sp2.fetchSample(sample2, 0);
			LCD.clear(4);
			LCD.drawString("Sonic: " + sample2[0],0,4);
			sp3.fetchSample(sample3, 0);
			LCD.clear(5);
			LCD.drawString("Gyro: " + sample3[0],0,5);
			Delay.msDelay((long) (1000/frequency));
		}
		
		irSensor.close();
		sonicSensor.close();
		gyroSensor.close();
	}
}
