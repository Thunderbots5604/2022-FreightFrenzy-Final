package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.hardware.TrebuchetArm;
import org.firstinspires.ftc.teamcode.hardware.mecanumdrive.MecanumAutonomous;
import org.firstinspires.ftc.teamcode.math.linearalgebra.TwoDimensionalVector;
import org.firstinspires.ftc.teamcode.pathing.RobotPosition;

@Autonomous (name = "JeffAuto", group = "Autonomous")
public class JeffAuto extends LinearOpMode {
    private MecanumAutonomous drive;
    private TrebuchetArm trebuchet;
    private Toggles toggles;
    private ElapsedTime timer;

    private RobotPosition[] positions = new RobotPosition[] {
            new RobotPosition(new TwoDimensionalVector(0, -2100), 0),
            new RobotPosition(new TwoDimensionalVector(-3800, 1000), 0),
            new RobotPosition(new TwoDimensionalVector(-3300, -3100), 0),
            
            
    };
    @Override
    public void runOpMode() throws InterruptedException {
        //set up
        drive = new MecanumAutonomous(hardwareMap, new RobotPosition(new TwoDimensionalVector(0, 0), 0), 35, 1000, 5, 180);
        trebuchet = new TrebuchetArm(hardwareMap, "trebuchet", "trebuchet2", 10000, 0, new int[]{0, 25, 50, 75, 100, 125, 150, 175, 200, 225}, 15, .015, 0.9, 0.1, 0.1, 0.1, 0.1);
        toggles = new Toggles(hardwareMap);
        timer = new ElapsedTime();

        waitForStart();
        
        drive.moveTo(positions[1]);
        timer.reset();
        while(timer.milliseconds()<5000){
            toggles.startFlywheel(true);
        }
        toggles.stopFlywheel();
        //put block in
        drive.moveTo(positions[1]);
        drive.moveTo(positions[2]);

    }
}
