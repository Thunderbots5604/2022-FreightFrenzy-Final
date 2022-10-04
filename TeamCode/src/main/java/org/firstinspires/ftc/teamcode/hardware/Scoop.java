package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Scoop {
    //servo
    private Servo scoopServo;
    private double upPosition;
    private double downPosition;

    //constructor
    public Scoop(HardwareMap hardwareMap, String scooper, double upPosition, double downPosition){
        scoopServo = hardwareMap.get(Servo.class, scooper);
        this.upPosition = upPosition;
        this.downPosition = downPosition;
    }

    public void up(){
        scoopServo.setPosition(upPosition);
    }

    public void down(){
        scoopServo.setPosition(downPosition);
    }
}
