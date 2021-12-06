package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

//This class lets the robot run much better while reading a distance sensor by using multithreading

public class ContinuousSensor2m extends Thread {
    private DistanceSensor Sensor;
    private double distance, refreshRate;


    public ContinuousSensor2m(DistanceSensor Sensor){
        this(Sensor, 30);
    }
    public ContinuousSensor2m(DistanceSensor Sensor, double refreshRate){
        this.Sensor = Sensor;
        this.refreshRate = refreshRate;
    }

    public void run(){
        while(true) {
            distance = Sensor.getDistance(DistanceUnit.METER);
            try {
                Thread.sleep((long) ((1/refreshRate)*1000));
            } catch (InterruptedException e) {
                System.out.println("Wow!");
            }
        }
    }

    public double getDistance(){
        return distance;
    }
}
