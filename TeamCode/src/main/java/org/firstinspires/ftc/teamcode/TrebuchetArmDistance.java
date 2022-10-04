package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;


public class TrebuchetArmDistance {
    public DistanceSensor distance;
    private DcMotorEx trebuchet;


    public void initialize(HardwareMap hardwareMap) {
        distance = hardwareMap.get(DistanceSensor.class, "distance");
        trebuchet = hardwareMap.get(DcMotorEx.class,"trebuchet");

        trebuchet.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        trebuchet.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    public void liftArm(double measuredDistance){
        if(measuredDistance < 0.2) {
            trebuchet.setTargetPosition(200);
        }
        else{
            trebuchet.setTargetPosition(0);
        }

    }
    // todo: write your code here
}