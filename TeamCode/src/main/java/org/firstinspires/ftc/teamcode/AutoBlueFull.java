package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.hardware.Distance;
import org.firstinspires.ftc.teamcode.hardware.TrebuchetArm;
import org.firstinspires.ftc.teamcode.hardware.mecanumdrive.MecanumAutonomous;
import org.firstinspires.ftc.teamcode.math.linearalgebra.TwoDimensionalVector;
import org.firstinspires.ftc.teamcode.pathing.RobotPosition;

@Autonomous(name = "Blue Side Full Autonomous", group = "Autonomous")
public class AutoBlueFull extends LinearOpMode {
    private RobotPosition[] positions = new RobotPosition[] {
            new RobotPosition(new TwoDimensionalVector(0, -200), 0),
            new RobotPosition(new TwoDimensionalVector(3750, -200), 0),
            new RobotPosition(new TwoDimensionalVector(1900, -950), 0),
            new RobotPosition(new TwoDimensionalVector(500, -950), 0),
            new RobotPosition(new TwoDimensionalVector(-200, -950), 0),
            new RobotPosition(new TwoDimensionalVector(-1000, -950), 0),
            new RobotPosition(new TwoDimensionalVector(-1000, -25), 90),
            new RobotPosition(new TwoDimensionalVector(-2000, -25), 90),
            new RobotPosition(new TwoDimensionalVector(-200, -750), 90)
    };

    private MecanumAutonomous drive;
    private Distance distance;
    private int setUp = 3;
    private double cmAway = 30;
    private TrebuchetArm trebuchet;
    private int trebuchetPosition = 17;
    private ElapsedTime timer;
    private Toggles toggles;

    @Override
    public void runOpMode() throws InterruptedException {
        //set up
        drive = new MecanumAutonomous(hardwareMap, new RobotPosition(new TwoDimensionalVector(0, 0), 0), 35, 3000, 1, 180);
        distance = new Distance(hardwareMap, "distance");
        trebuchet = new TrebuchetArm(hardwareMap, "trebuchet", "trebuchet2", /*"rotate",*/ 10000, 0, new int[] {0, 25, 50, 75, 100, 125, 150, 175, 200, 225}, 15, .015, 0.4, 0.1, 0.1, 0.1, 0.1);
        timer = new ElapsedTime();
        toggles = new Toggles(hardwareMap);

        waitForStart();
        
        toggles.startFlywheel(true);
        drive.moveTo(positions[0]);

        //go to 1
        drive.moveTo(positions[1]);
        timer.reset();
        while(timer.milliseconds() < 5000) {}
        toggles.stopFlywheel();

        for(int i = 1; i <= 3; i++) {
            drive.moveTo(positions[i+1]);
            if(distance.centimeters() < cmAway) {
                setUp = i;
                switch(i) {
                    case 1:
                        trebuchetPosition = 17;
                        break;
                    case 2:
                        trebuchetPosition = 14;
                        break;
                    case 3:
                        trebuchetPosition = 11;
                        break;
                }
                break;
            }
            sleep(100);
        }
        telemetry.addData("SetUp", setUp);
        telemetry.update();

        //go to 5
        drive.moveTo(positions[5]);
        timer.reset();
        while(timer.milliseconds() < 3000) {
            trebuchet.moveToTarget(100);
        }
        while(timer.milliseconds() < 5000) {
            trebuchet.moveToTarget(100);
            toggles.flipIntake();
        }
        toggles.stopIntake();
        trebuchet.setCurrentPosition(0);
        timer.reset();
        while(timer.milliseconds() < 3000) {
            trebuchet.moveToTarget(100);
        }

        //go to 6
        drive.moveTo(positions[6]);

        //go to 7
        drive.moveTo(positions[7]);

        //go to 8
        drive.moveTo(positions[8]);
    }
}
