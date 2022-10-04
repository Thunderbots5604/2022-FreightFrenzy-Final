package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.hardware.TrebuchetArm;
import org.firstinspires.ftc.teamcode.hardware.mecanumdrive.MecanumAutonomous;
import org.firstinspires.ftc.teamcode.math.linearalgebra.TwoDimensionalVector;
import org.firstinspires.ftc.teamcode.pathing.RobotPosition;

@Autonomous(name = "Duck and Park Red", group = "Autonomous")
public class AutoDuckParkRed extends LinearOpMode{

    private RobotPosition[] positions = new RobotPosition[] {
            new RobotPosition(new TwoDimensionalVector(0, -200), 0),
            new RobotPosition(new TwoDimensionalVector(-3750, -200), 0),
            new RobotPosition(new TwoDimensionalVector(-3750, -200), -60),
            new RobotPosition(new TwoDimensionalVector(1000, -12000), -60)
    };

    private MecanumAutonomous drive;
    private int setUp = 3;
    private double cmAway = 30;
    private TrebuchetArm trebuchet;
    private int trebuchetPosition = 17;
    private ElapsedTime timer;
    private Toggles toggles;
    
    @Override
    public void runOpMode() {
        drive = new MecanumAutonomous(hardwareMap, new RobotPosition(new TwoDimensionalVector(0, 0), 0), 35, 3000, 1, 180);
        trebuchet = new TrebuchetArm(hardwareMap, "trebuchet", "trebuchet2", /*"rotate",*/ 10000, 0, new int[] {0, 25, 50, 75, 100, 125, 150, 175, 200, 225}, 15, .015, 0.4, 0.1, 0.1, 0.1, 0.1);
        timer = new ElapsedTime();
        toggles = new Toggles(hardwareMap);

        waitForStart();
        
        drive.moveTo(positions[0]);
        
        toggles.startFlywheel(true);

        //go to 1
        drive.moveTo(positions[1]);
        timer.reset();
        while(timer.milliseconds() < 5000) {}
        toggles.stopFlywheel();
        
        drive.moveTo(positions[2]);
        
        drive.moveTo(positions[3]);
    }
}