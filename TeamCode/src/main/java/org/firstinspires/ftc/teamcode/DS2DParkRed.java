package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.hardware.mecanumdrive.MecanumAutonomous;
import org.firstinspires.ftc.teamcode.math.linearalgebra.TwoDimensionalVector;
import org.firstinspires.ftc.teamcode.pathing.RobotPosition;

@Autonomous(name = "Duck Side to Depot Park Red (DS)", group = "Autonomous")
public class DS2DParkRed extends LinearOpMode {
    private MecanumAutonomous drive;
    private RobotPosition[] positions = new RobotPosition[] {
            new RobotPosition(new TwoDimensionalVector(3300, -3100), 0),
    };

    enum DrivePosition {
        DRIVEPOSITION_START,
        DRIVEPOSITION_TODEPOT
    }

    volatile DrivePosition drivePosition = DrivePosition.DRIVEPOSITION_START;

    @Override
    public void runOpMode() {
        drive = new MecanumAutonomous(hardwareMap, new RobotPosition(new TwoDimensionalVector(0, 0), 0), 50, 1000, 10, 180);
        while (opModeIsActive()) {
           switch (drivePosition) {
               case DRIVEPOSITION_START:
                   drivePosition = DrivePosition.DRIVEPOSITION_TODEPOT;
                   break;
               case DRIVEPOSITION_TODEPOT:
                   if (drive.moveTo(positions[0])) {
                       drivePosition = DrivePosition.DRIVEPOSITION_TODEPOT;
                   }
                   break;
               default:
                   drivePosition = DrivePosition.DRIVEPOSITION_START;
           }
        }
    }
}
