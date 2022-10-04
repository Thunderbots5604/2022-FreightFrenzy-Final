package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name="Motor test", group="test")

public class Delete extends OpMode {
    
    private DcMotor frontLeftMotor;
    private DcMotor frontRightMotor;
    private DcMotor backLeftMotor;
    private DcMotor backRightMotor;
    private DcMotor trebuchet1;
    private DcMotor trebuchet2;
    
    public void init() {
        //get motors set up
        frontLeftMotor = hardwareMap.get(DcMotor.class, "lmf");
        frontRightMotor = hardwareMap.get(DcMotor.class, "rmf");
        backLeftMotor = hardwareMap.get(DcMotor.class, "lmb");
        backRightMotor = hardwareMap.get(DcMotor.class, "rmb");
        trebuchet1 = hardwareMap.get(DcMotor.class, "trebuchet");
        trebuchet2 = hardwareMap.get(DcMotor.class, "trebuchet2");

        frontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        trebuchet1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        trebuchet2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        trebuchet1.setDirection(DcMotorSimple.Direction.REVERSE);
        
    }
    
    public void start() {
        
    }

    public void loop() {
        if(gamepad1.a) {
            frontLeftMotor.setPower(0.5);
        }
        else {
            frontLeftMotor.setPower(0);
        }
        if(gamepad1.b) {
            frontRightMotor.setPower(0.5);
        }
        else {
            frontRightMotor.setPower(0);
        }
        if(gamepad1.x) {
            backLeftMotor.setPower(0.5);
        } 
        else {
            backLeftMotor.setPower(0);
        }
        if(gamepad1.y) {
            backRightMotor.setPower(0.5);
        }
        else {
            backRightMotor.setPower(0);
        }
        if(gamepad1.right_bumper) {
            trebuchet1.setPower(-1);
            trebuchet2.setPower(-1);
        }
        else if(gamepad1.left_bumper) {
            trebuchet1.setPower(1);
            trebuchet2.setPower(1);
        }
        else {
            trebuchet1.setPower(0);
            trebuchet2.setPower(0);
        }

    }
}