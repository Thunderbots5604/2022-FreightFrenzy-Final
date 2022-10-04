package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.hardware.mecanumdrive.MecanumAutonomous;
import org.firstinspires.ftc.teamcode.math.linearalgebra.TwoDimensionalVector;
import org.firstinspires.ftc.teamcode.pathing.RobotPosition;

@Autonomous(name = "Warehouse Side to Warehouse Park Blue (DS)", group = "Autonomous")
public class WS2WHBlue extends LinearOpMode {
    private MecanumAutonomous drive;
    private RobotPosition[] positions = new RobotPosition[] {
            new RobotPosition(new TwoDimensionalVector(100, -500), 0),
            new RobotPosition(new TwoDimensionalVector(-2600, 1200), 0)
    };

    enum DrivePosition {
        DRIVEPOSITION_START,
        DRIVEPOSITION_TOWH,
        DRIVEPOSITION_TOPARK,
        DRIVEPOSITION_PARK
    }

    volatile DrivePosition drivePosition = DrivePosition.DRIVEPOSITION_START;

    @Override
    public void runOpMode() {
        drive = new MecanumAutonomous(hardwareMap, new RobotPosition(new TwoDimensionalVector(0, 0), 0), 50, 1000, 10, 180);
        while (opModeIsActive()) {
           switch (drivePosition) {
               case DRIVEPOSITION_START:
                   drivePosition = DrivePosition.DRIVEPOSITION_TOWH;
                   break;
               case DRIVEPOSITION_TOWH:
                   if (drive.moveTo(positions[0])) {
                       drivePosition = DrivePosition.DRIVEPOSITION_TOPARK;
                   }
                   break;
               case DRIVEPOSITION_TOPARK:
                   if (drive.moveTo(positions[1])) {
                       drivePosition = DrivePosition.DRIVEPOSITION_PARK;
                   }
                   break;
               case DRIVEPOSITION_PARK:
                   stop();
                   break;
               default:
                   drivePosition = DrivePosition.DRIVEPOSITION_START;
           }
        }
    }
}
