package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.hardware.mecanumdrive.MecanumDriveLibra;
import org.firstinspires.ftc.teamcode.math.linearalgebra.TwoDimensionalVector;

@TeleOp(name="Drive Train Test", group="real anal(ysis)")
public class MotorTest extends OpMode {
    public MecanumDriveLibra driveTrain;
    @Override
    public void init() {
        driveTrain = new MecanumDriveLibra(hardwareMap, "lmf", "rmf", "lmb", "rmb");
    }

    @Override
    public void loop() {
        if(gamepad1.a) {
            driveTrain.linearPowerCalculations(new TwoDimensionalVector(-0.5, -0.5));
            driveTrain.pushPowers();
        }
        else {
            driveTrain.stop();
        }
        if(gamepad1.b) {
            driveTrain.resetEncoders();
        }
        telemetry.addData("fl Ticks", driveTrain.getFrontLeftTicks());
        telemetry.addData("fr Ticks", driveTrain.getFrontRightTicks());
        telemetry.addData("bl ticks", driveTrain.getBackLeftTicks());
        telemetry.addData("br ticks", driveTrain.getBackRightTicks());
        telemetry.update();
    }
}
