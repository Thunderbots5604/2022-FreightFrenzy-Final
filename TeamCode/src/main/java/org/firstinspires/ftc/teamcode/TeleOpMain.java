package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.hardware.Gyro;
import org.firstinspires.ftc.teamcode.hardware.TrebuchetArm;
import org.firstinspires.ftc.teamcode.hardware.mecanumdrive.MecanumTeleOp;

@TeleOp(name = "TeleOpMain", group = "TeleOp")
public class TeleOpMain extends OpMode {

    private Gyro gyro = new Gyro();

    //private Linears linears = new Linears(telemetry);
    private TrebuchetArm trebuchetArm;
    private Toggles toggles;

    private MecanumTeleOp drive;
    private double[] drivePowers;
    private double armPosition;

    private double multiplier = 1;
    private double leftTrigger = 0;
    private double rightTrigger = 0;
    private double quarterSpeed = multiplier * 0.25;

    private boolean reverse = false;
    private boolean ring = false;
    private boolean pastX = false;
    private boolean secondX = false;
    private boolean pastY = false;
    private boolean pastB = false;
    private boolean secondB = false;
    private boolean pastA = false;
    private boolean secondA = false;
    private boolean pastLB = false;
    private boolean pastRB = false;
    private boolean engageArm = false;
    private boolean engageLock = false;
    private boolean swap = false;
    private boolean pastBack = false;
    private boolean engageRing = false;
    private boolean pastDUp = false;
    private boolean pastDRight = false;
    private boolean pastDDown = false;
    private boolean pastDLeft = false;
    private boolean past2X = false;
    private boolean allOff = false;
    private boolean scoopChanged = false;
    private boolean flywheelChanged = false;
    private boolean intakeStarted = false;
    private boolean pressedB = false;
    private boolean halfSpeed = false;
    private boolean fixArm = false;

    /*
    private MecanumDrive drive;
    private Point direction;
    private double power;
    */

    @Override
    public void init() {
        drive = new MecanumTeleOp(hardwareMap, 1);
        trebuchetArm = new TrebuchetArm(hardwareMap);
        toggles = new Toggles(hardwareMap);
        telemetry.addData("Status", "initialized");
        telemetry.update();
    }

    public void start() {
    }

    @Override
    public void loop() {
        boolean pressedBack = gamepad1.back;
        if (pressedBack && !pastBack) {
            swap = !swap;
        }
        pastBack = pressedBack;
        if (swap) {
            telemetry.addData("flywheel direction", "clockwise");
        } else {
            telemetry.addData("flywheel direction", "counterclockwise");
        }

        leftTrigger = gamepad2.left_trigger;
        rightTrigger = gamepad2.right_trigger;

        if(gamepad1.dpad_up && !pastDUp) {
            drive.orient(1);
        }
        else if (gamepad1.dpad_right && !pastDRight) {
            drive.orient(0);
        }
        else if (gamepad1.dpad_down && !pastDDown) {
            drive.orient(3);
        }
        else if (gamepad1.dpad_left && !pastDLeft) {
            drive.orient(2);
        }
        pastDUp = gamepad1.dpad_up;
        pastDRight = gamepad1.dpad_right;
        pastDDown = gamepad1.dpad_down;
        pastDLeft = gamepad1.dpad_left;

        //run the teleop stuff and store the powers
        multiplier = 1;
        if (halfSpeed) {
            multiplier *= .5;
        }
        drive.move(multiplier * gamepad1.left_stick_x, multiplier * gamepad1.left_stick_y, multiplier * gamepad1.left_trigger / 2 - multiplier * gamepad1.right_trigger / 2);
        drivePowers = drive.getMotorPowers();
        //put the stuff into telemetry
        telemetry.addData("Status", "running");
        telemetry.addData("fl power", "%,.3f", drivePowers[0]);
        telemetry.addData("fr power", "%,.3f", drivePowers[1]);
        telemetry.addData("bl power", "%,.3f", drivePowers[2]);
        telemetry.addData("br power", "%,.3f", drivePowers[3]);



        if (gamepad1.right_trigger > 0.1) {
            trebuchetArm.setPower(gamepad1.right_trigger);
            fixArm = false;
        } else if (gamepad1.left_trigger > 0.1) {
            trebuchetArm.setPower(-gamepad1.left_trigger);
            fixArm = false;
        } 
        else if (fixArm) {
            trebuchetArm.setPower(.12);
        }
        else {
            trebuchetArm.stop();
        }
        armPosition = trebuchetArm.armPosition;
        
         //move arm by 45 degree incriments (change the number added to that it will work with tics.)
        /*if(gamepad1.left_bumper == true){
            trebuchetArm.setArmAngle(trebuchetArm.getCurrentPosition() + 45);
        }
        else if(gamepad1.right_bumper == true){
            trebuchetArm.setArmAngle(trebuchetArm.getCurrentPosition() - 45);
        } else{
            trebuchetArm.stopRotate();
        }*/
        telemetry.addData("trebuchet arm position", "%,.3f", armPosition);
        boolean pressedX = gamepad1.x;
        if (pressedX && !pastX) {
            if (flywheelChanged) {
                toggles.stopFlywheel();
                flywheelChanged = false;
            }
            else {
                toggles.startFlywheel(swap);
                flywheelChanged = true;
            }
        }
        pastX = pressedX;
        boolean pressedY = gamepad1.y;
        if (pressedY && !pastY) {
            fixArm = !fixArm;
        }
        pastY = pressedY;
        boolean pressedB = gamepad1.b;
        if (pressedB && !pastB) {
            if (intakeStarted) {
                intakeStarted = false;
                toggles.stopIntake();
            } else {
                intakeStarted = true;
                toggles.flipIntake();
            }
        }
        pastB = pressedB;
        boolean pressedA = gamepad1.a;
        if (pressedA && !pastA) {
            if (intakeStarted) {
                intakeStarted = false;
                toggles.stopIntake();
            } else {
                intakeStarted = true;
                toggles.startIntake();
            }
        }
        pastA = pressedA;
        telemetry.update();
    }

    public void stop() {
        //Turn Off Motors
        drive.stop();
        trebuchetArm.stop();
        //toggles.stopIntake();
        toggles.stopFlywheel();
    }
}