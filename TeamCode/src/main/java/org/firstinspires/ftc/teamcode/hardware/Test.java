package org.firstinspires.ftc.teamcode.hardware;// package org.firstinspires.ftc.teamcode;

// import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
// import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
// import com.qualcomm.robotcore.util.ElapsedTime;

// import org.firstinspires.ftc.teamcode.hardware.Distance;
// import org.firstinspires.ftc.teamcode.hardware.TrebuchetArm;
// import org.firstinspires.ftc.teamcode.hardware.mecanumdrive.MecanumAutonomous;
// import org.firstinspires.ftc.teamcode.math.linearalgebra.TwoDimensionalVector;
// import org.firstinspires.ftc.teamcode.pathing.RobotPosition;
// import org.firstinspires.ftc.teamcode.hardware.IntakeSensor;


// @Autonomous(name = "Test", group = "Test")
// public class Test extends LinearOpMode {
//     private RobotPosition[] positions = new RobotPosition[] {
//             new RobotPosition(new TwoDimensionalVector(0, -200), 0),
//             new RobotPosition(new TwoDimensionalVector(3750, -200), 0),
//             new RobotPosition(new TwoDimensionalVector(1900, -950), 0),
//             new RobotPosition(new TwoDimensionalVector(500, -950), 0),
//             new RobotPosition(new TwoDimensionalVector(-200, -950), 0),
//             new RobotPosition(new TwoDimensionalVector(-1000, -950), 0),
//             new RobotPosition(new TwoDimensionalVector(-1000, -25), 90),
//             new RobotPosition(new TwoDimensionalVector(-2000, -25), 90),
//             new RobotPosition(new TwoDimensionalVector(-200, -750), 90)
//     };

//     private MecanumAutonomous drive;
//     private Distance distance;
//     private IntakeSensor intakeSensor;
//     private int setUp = 3;
//     private double cmAway = 30;
//     private TrebuchetArm trebuchet;
//     private int trebuchetPosition = 17;
//     private ElapsedTime timer;
//     private Toggles toggles;

//     @Override
//     public void runOpMode() throws InterruptedException {
//         //set up
//         drive = new MecanumAutonomous(hardwareMap, new RobotPosition(new TwoDimensionalVector(0, 0), 0), 35, 3000, 1, 180);
//         intakeSensor = new IntakeSensor(hardwareMap, "intakeSensor");
//         trebuchet = new TrebuchetArm(hardwareMap, "trebuchet", "trebuchet2", /*"rotate",*/ 10000, 0, new int[] {0, 25, 50, 75, 100, 125, 150, 175, 200, 225}, 15, .015, 0.4, 0.1, 0.1, 0.1, 0.1);
//         timer = new ElapsedTime();
//         toggles = new Toggles(hardwareMap);

//         intakeSensor.run();
        
//         waitForStart();
        
//     }
// }
