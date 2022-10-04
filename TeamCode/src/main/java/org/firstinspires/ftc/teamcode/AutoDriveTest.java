package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.hardware.mecanumdrive.MecanumAutonomous;
import org.firstinspires.ftc.teamcode.math.linearalgebra.TwoDimensionalVector;
import org.firstinspires.ftc.teamcode.pathing.RobotPosition;

@Autonomous(name = "Drive Test", group = "test")
public class AutoDriveTest extends LinearOpMode {
    double targetX = 100;
    double targetY = 0;
    double targetAngle = 90;
    MecanumAutonomous drive;
    /*Distance distance;
    FindCapstonePosition camera;
    FindCapstonePosition.CapstonePosition position;*/
    @Override
    public void runOpMode() {
        drive = new MecanumAutonomous(hardwareMap, new RobotPosition(new TwoDimensionalVector(0, 0), 0), 50, 1000, 10, 180);
        //camera = new FindCapstonePosition(hardwareMap, telemetry, this);
        //camera.init();
        //position = camera.passPosition();
        waitForStart();
        
        drive.targetingTest(new RobotPosition(new TwoDimensionalVector(targetX, targetY), targetAngle), telemetry);
        drive.targetingTest(new RobotPosition(new TwoDimensionalVector(0, 0), 0), telemetry);
        
        sleep(1000000);
    }
}
