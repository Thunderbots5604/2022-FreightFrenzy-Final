package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.autonomous.FindCapstonePosition;

@Autonomous(name="Camera Test",group="test")
public class CameraTest extends LinearOpMode {
    FindCapstonePosition camera;
    @Override
    public void runOpMode() {
        camera = new FindCapstonePosition(hardwareMap, telemetry, this);
        camera.init();
        waitForStart();
        while (true) {
            camera.findRegion();
            telemetry.update();
            if (!opModeIsActive()) {
                break;
            }
        }
    }
}