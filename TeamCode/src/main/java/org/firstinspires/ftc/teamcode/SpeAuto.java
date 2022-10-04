package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.hardware.Distance;
import org.firstinspires.ftc.teamcode.hardware.TrebuchetArm;
import org.firstinspires.ftc.teamcode.hardware.mecanumdrive.MecanumAutonomous;
import org.firstinspires.ftc.teamcode.math.linearalgebra.TwoDimensionalVector;
import org.firstinspires.ftc.teamcode.pathing.RobotPosition;

@Autonomous (name = "SpencerValueTest", group = "Autonomous")
public class SpeAuto extends LinearOpMode {
    private MecanumAutonomous drive;
    private Distance distance;
    private int setUp = 3;
    private double cmAway = 100;
    private TrebuchetArm trebuchet;
    private int trebuchetPosition = 17;
    private ElapsedTime timer;
    private Toggles toggles;

    private RobotPosition[] positions = new RobotPosition[] {
            new RobotPosition(new TwoDimensionalVector(-2550, -2550), 0),
            new RobotPosition(new TwoDimensionalVector(100, -500), 0),
            new RobotPosition(new TwoDimensionalVector(130, 3500), 0),
            new RobotPosition(new TwoDimensionalVector(130, -1500), 0),
            new RobotPosition(new TwoDimensionalVector(-2550, -2550), 0),
            new RobotPosition(new TwoDimensionalVector(120, -500), 0),
            new RobotPosition(new TwoDimensionalVector(170, 3700), 0),
            new RobotPosition(new TwoDimensionalVector(170, -1500), 0),
            new RobotPosition(new TwoDimensionalVector(-2350, -2350), 0),
            new RobotPosition(new TwoDimensionalVector(110, 0), 0),
            new RobotPosition(new TwoDimensionalVector(140, 2500), 0),
            new RobotPosition(new TwoDimensionalVector(140, -500), 0),
            new RobotPosition(new TwoDimensionalVector(-2350, -2350), 0),
            new RobotPosition(new TwoDimensionalVector(0, 0), 0),
            new RobotPosition(new TwoDimensionalVector(0, 2500), 0),
            new RobotPosition(new TwoDimensionalVector(0, -500), 0),
            new RobotPosition(new TwoDimensionalVector(-500, -500), 0),
            new RobotPosition(new TwoDimensionalVector(0, 0), 0),
            new RobotPosition(new TwoDimensionalVector(0, 2000), 0),



    };
    @Override
    public void runOpMode() throws InterruptedException {
        //set up
        drive = new MecanumAutonomous(hardwareMap, new RobotPosition(new TwoDimensionalVector(0, 0), 0), 35, 3000, 1, 180);
        trebuchet = new TrebuchetArm(hardwareMap, "trebuchet", "trebuchet2", 10000, 0, new int[]{0, 25, 50, 75, 100, 125, 150, 175, 200, 225}, 5, 1, 700, 500, 0.1, 0.1, 0.1);
        timer = new ElapsedTime();
        toggles = new Toggles(hardwareMap);

        waitForStart();
        
        timer.reset();
        
        while(true){
          trebuchet.moveToTarget(-125);
          //values:
          //low: 50
          //mid: 125
          //high: 175
        }
    }
}
