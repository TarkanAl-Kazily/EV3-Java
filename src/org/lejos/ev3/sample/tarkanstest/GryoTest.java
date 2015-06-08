/*
 * Prints out the angle of the gyroscope to the screen. The gyro must be
 * connected to sensor port 2.
 */

package org.lejos.ev3.sample.tarkanstest;

import lejos.hardware.BrickFinder;
import lejos.hardware.lcd.Font;
import lejos.hardware.lcd.GraphicsLCD;
import lejos.hardware.port.SensorPort;
import lejos.hardware.sensor.EV3GyroSensor;
import lejos.hardware.sensor.SensorModes;
import lejos.robotics.SampleProvider;
import lejos.utility.Delay;

public class GryoTest {

	static GraphicsLCD g;
	static int SW;
	static int SH;
	
	static SensorModes gyro;
	static SampleProvider rotation;
	static float[] sample;
	
	public static void main(String[] args) {
		g = BrickFinder.getDefault().getGraphicsLCD();
        SW = g.getWidth();
        SH = g.getHeight();
        g.setFont(Font.getSmallFont());
        
        gyro = new EV3GyroSensor(SensorPort.S2);
        rotation = gyro.getMode("Angle");
        sample = new float[rotation.sampleSize()];
        while (true) {
        	if ((rotation != null) && (sample != null)) {
        		rotation.fetchSample(sample, 0);
        		print(Float.toString(sample[0]));
        		sample[0] = 0;
        	}
        	Delay.msDelay(10);
        }
	}

	public static void print(String word){
		g.clear();
        g.drawString(word, SW/2, SH/2, GraphicsLCD.BASELINE|GraphicsLCD.HCENTER);
	}
	
}
