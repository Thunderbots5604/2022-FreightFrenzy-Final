package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.autonomous.AutonomousMove;
import org.firstinspires.ftc.teamcode.hardware.TrebuchetArm;
import org.firstinspires.ftc.teamcode.hardware.mecanumdrive.MecanumDriveLibra;
import org.firstinspires.ftc.teamcode.math.Values;

@Autonomous(name="FlywheelAutoRed", group="Red")
public class FlywheelAutoRed extends LinearOpMode {
    private int tileLength = Values.tileLength;
    private double ticksPerDegree = Values.ticksPerDegree;
    private double power = Values.power;
    private boolean direction = false;
    @Override
    public void runOpMode() {
        TrebuchetArm trebuchetArm = new TrebuchetArm(hardwareMap);
        Toggles toggles = new Toggles(hardwareMap);
        MecanumDriveLibra drive = new MecanumDriveLibra(hardwareMap, "lmf", "rmf", "lmb", "rmb");
        AutonomousMove move = new AutonomousMove(hardwareMap, telemetry, this);
        telemetry.addData("Status", "running");
        telemetry.update();
        move.strafeLeft(tileLength, power);
        sleep(500);
        toggles.startFlywheel(direction);
        sleep(2000);
        toggles.stopFlywheel();
        sleep(500);
        move.move(0, power, "gyro turn");
        sleep(500);
        move.turnLeft(ticksPerDegree * 90, power);
    }
}
