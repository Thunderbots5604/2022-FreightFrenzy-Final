package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Distance {
    private DistanceSensor sensor;
    private ElapsedTime timer;

    public Distance(HardwareMap map, String sensorName) {
        this.sensor = map.get(DistanceSensor.class, sensorName);
        this.timer = new ElapsedTime();
    }

    public double centimeters() {
        double cm = 0;
        timer.reset();
        while(cm == 0 && timer.milliseconds() < 1500) {
            cm = sensor.getDistance(DistanceUnit.CM);
        }
        if(cm == 0) {
            return 10000;
        }
        else {
            return cm;
        }
    }
}
