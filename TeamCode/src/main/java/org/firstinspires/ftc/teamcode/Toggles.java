package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Disabled
public class Toggles {

    private CRServo flywheel;
    private DcMotorEx intake;
    //private DcMotorEx scoop;

    private LinearOpMode opmode;
    private Telemetry telemetry;

    private int startingPosition;

    public Toggles(HardwareMap hardwareMap) {
        flywheel = hardwareMap.get(CRServo.class, "flywheel");
        intake = hardwareMap.get(DcMotorEx.class, "intake");
        //scoop = hardwareMap.get(DcMotorEx.class, "scoop");

        intake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        /*
        scoop.setTargetPosition(startingPosition);
        scoop.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        scoop.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        scoop.setTargetPositionTolerance(5);
        startingPosition = scoop.getCurrentPosition();
        */
    }
    public void startFlywheel(boolean direction) {
        if (direction) {
            flywheel.setPower(1);
        } else {
            flywheel.setPower(-1);
        }
    }
    public void startIntake() {
        intake.setPower(-1.0);
    }
    public void flipIntake() {
        intake.setPower(0.35);
    }
    public void stopFlywheel() {
        flywheel.setPower(0);
    }
    public void stopIntake() {
        intake.setPower(0);
    }
    /*
    public void scoopUp() {
        scoop.setVelocity(900);
        scoop.setTargetPosition(startingPosition + 300);
    }
    public void scoopDown() {
        scoop.setVelocity(-900);
        scoop.setTargetPosition(startingPosition);
    }
    */
}
