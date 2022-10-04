package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.autonomous.AutonomousMove;
import org.firstinspires.ftc.teamcode.hardware.TrebuchetArm;
import org.firstinspires.ftc.teamcode.hardware.mecanumdrive.MecanumDriveLibra;
import org.firstinspires.ftc.teamcode.math.Values;

@Autonomous(name="FlywheelAutoBlue", group="Blue")
public class FlywheelAutoBlue extends LinearOpMode {
    private int tileLength = Values.tileLength;
    private double ticksPerDegree = Values.ticksPerDegree;
    private double power = Values.power;
    private boolean direction = true;
    @Override
    public void runOpMode() {
        TrebuchetArm trebuchetArm = new TrebuchetArm(hardwareMap);
        Toggles toggles = new Toggles(hardwareMap);
        MecanumDriveLibra drive = new MecanumDriveLibra(hardwareMap, "lmf", "rmf", "lmb", "rmb");
        AutonomousMove move = new AutonomousMove(hardwareMap, telemetry, this);
        telemetry.addData("Status", "running");
        telemetry.update();
        move.strafeRight(tileLength, power);
        sleep(500);
        move.turnRight(ticksPerDegree * 135, power);
        sleep(500);
        toggles.startFlywheel(direction);
        sleep(2000);
        toggles.stopFlywheel();
        sleep(500);
        move.move(0, power, "gyro turn");
        sleep(500);
        move.turnLeft(ticksPerDegree * 90, power);
        move.moveForward(tileLength * 0.3, power * 0.5);
    }
}
