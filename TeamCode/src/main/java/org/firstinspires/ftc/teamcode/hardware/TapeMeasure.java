package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;


public class TapeMeasure {
    private CRServo pitch1;
    private CRServo pitch2;
    private CRServo yaw1;
    private CRServo yaw2;
    private CRServo extend;
    
    public TapeMeasure(HardwareMap map, String pitch1, String pitch2, String yaw1, String yaw2, String extend){
        this.pitch1 = map.get(CRServo.class, pitch1);
        this.pitch2 = map.get(CRServo.class, pitch2);
        this.yaw1 = map.get(CRServo.class, yaw1);
        this.yaw2 = map.get(CRServo.class, yaw2);
        this.extend = map.get(CRServo.class, extend);
    }
    
    public void pitchPower(double power) {
        pitch1.setPower(power);
        pitch2.setPower(power);
    }
    
    public void yawPower(double power) {
        yaw1.setPower(power);
        yaw2.setPower(-power);
    }
    
    public void extendPower(double power) {
        extend.setPower(power);
    }
}