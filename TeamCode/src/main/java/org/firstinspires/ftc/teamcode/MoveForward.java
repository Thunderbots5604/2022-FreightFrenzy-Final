package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.hardware.mecanumdrive.MecanumDriveLibra;
import org.firstinspires.ftc.teamcode.math.linearalgebra.TwoDimensionalVector;

@Autonomous(name="Move Forward")
public class MoveForward extends LinearOpMode {
    private MecanumDriveLibra drive;
    private int howMuch = 2500;
    private ElapsedTime timer;

    @Override
    public void runOpMode() {
        timer = new ElapsedTime();
        drive = new MecanumDriveLibra(hardwareMap, "lmf", "rmf", "lmb", "rmb", false);

        drive.resetEncoders();
        
        waitForStart();

        timer.reset();
        
        while((Math.abs(drive.getFrontLeftTicks()) < howMuch || Math.abs(drive.getFrontRightTicks()) < howMuch || Math.abs(drive.getBackLeftTicks()) < howMuch || Math.abs(drive.getBackRightTicks()) < howMuch) && timer.milliseconds() < 10000) {
            drive.powersToZero();
            drive.linearPowerCalculations(new TwoDimensionalVector(0.05, .75));
            drive.pushPowers();
        }
    }

}
