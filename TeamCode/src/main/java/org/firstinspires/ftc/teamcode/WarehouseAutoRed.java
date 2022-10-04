package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.autonomous.AutonomousMove;
import org.firstinspires.ftc.teamcode.hardware.TrebuchetArm;
import org.firstinspires.ftc.teamcode.hardware.mecanumdrive.MecanumDriveLibra;
import org.firstinspires.ftc.teamcode.math.Values;

@Autonomous(name="WarehouseAutoRed", group="Red")
public class WarehouseAutoRed extends LinearOpMode {
    private int tileLength = Values.tileLength;
    private double ticksPerDegree = Values.ticksPerDegree;
    private double power = Values.power;
    @Override
    public void runOpMode() {
        TrebuchetArm trebuchetArm = new TrebuchetArm(hardwareMap);
        Toggles toggles = new Toggles(hardwareMap);
        MecanumDriveLibra drive = new MecanumDriveLibra(hardwareMap, "lmf", "rmf", "lmb", "rmb");
        AutonomousMove move = new AutonomousMove(hardwareMap, telemetry, this);
        telemetry.addData("Status", "running");
        telemetry.update();
        move.move(0, power, "gyro turn");
        sleep(500);
        move.turnRight(ticksPerDegree * 90, power);
        sleep(500);
        move.strafeRight(tileLength * 0.5, power * 0.5);
        sleep(500);
        move.moveForward(tileLength * 1.5, power);
    }
}
