package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.hardware.Gyro;
import org.firstinspires.ftc.teamcode.hardware.TapeMeasure;
import org.firstinspires.ftc.teamcode.hardware.TrebuchetArm;
import org.firstinspires.ftc.teamcode.hardware.mecanumdrive.MecanumTeleOp;

@TeleOp(name = "TestGyro", group = "TeleOp")
public class TestGyro extends OpMode {

    private Gyro gyro = new Gyro();

    //private Linears linears = new Linears(telemetry);
    private TrebuchetArm trebuchetArm;
    private Toggles toggles;
    private TapeMeasure tape;

    private MecanumTeleOp drive;
    private double[] drivePowers;
    private double armPosition;
    private DcMotor turret;

    private double multiplier = 1;
    private double leftTrigger = 0;
    private double rightTrigger = 0;
    private double quarterSpeed = multiplier * 0.25;

    private double angleMeasure = 0;

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
    private boolean pastRightBumper = false;
    private boolean pastLeftBumper = false;
    private boolean pastRightTrigger = false;
    private boolean pastLeftTrigger = false;
    private boolean pastRightYZero = false;

    private double gyroZero = 0;

    @Override
    public void init() {
        drive = new MecanumTeleOp(hardwareMap, 1);
        trebuchetArm = new TrebuchetArm(hardwareMap, "trebuchet", "trebuchet2", /*"rotate",*/ 10000, 0, new int[] {0, 50, 100, 150, 200, 250, 300, 350, 400, 450 /*775, 800, 825, 850, 875, 900, 925*/}, 5, .015, 0.4, 0.1, 0.1, 0.1, 0.1);
        toggles = new Toggles(hardwareMap);
        gyro.initGyro(hardwareMap);
        turret = hardwareMap.get(DcMotor.class, "turret");
        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        tape = new TapeMeasure(hardwareMap, "pitch1", "pitch2", "yaw1", "yaw2", "extend");
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
        if(gamepad1.dpad_up && !pastDUp) {
            drive.orient(0);
        }
        else if (gamepad1.dpad_right && !pastDRight) {
            drive.orient(1);
        }
        else if (gamepad1.dpad_down && !pastDDown) {
            drive.orient(2);
        }
        else if (gamepad1.dpad_left && !pastDLeft) {
            drive.orient(3);
        }
        pastDUp = gamepad1.dpad_up;
        pastDRight = gamepad1.dpad_right;
        pastDDown = gamepad1.dpad_down;
        pastDLeft = gamepad1.dpad_left;

        //run the teleop stuff and store the powers
        multiplier = 1;
        drive.move(multiplier * gamepad1.left_stick_x, multiplier * gamepad1.left_stick_y, multiplier * (gamepad1.left_trigger / 2 - gamepad1.right_trigger / 2));
        drivePowers = drive.getChangeInPositions();
        //put the stuff into telemetry
        telemetry.addData("Status", "running");
        telemetry.addData("fl ticks", "%,.3f", drivePowers[0]);
        telemetry.addData("fr ticks", "%,.3f", drivePowers[1]);
        telemetry.addData("bl ticks", "%,.3f", drivePowers[2]);
        telemetry.addData("br ticks", "%,.3f", drivePowers[3]);
        telemetry.addData("gyro angle", angleMeasure);

        int turretRotation = turret.getCurrentPosition();
        if(gamepad1.right_stick_x > 0.5){
            turret.setPower(-gamepad1.right_stick_x * 0.75);
        } else if(gamepad1.right_stick_x < -0.5){
            turret.setPower(-gamepad1.right_stick_x * 0.75);
        } else{
            turret.setPower(0);
        }
        telemetry.addData("Turret rotation", turretRotation);

        double rightY = gamepad1.right_stick_y;
        /*boolean rightYZero = (rightY <= 0.1 && rightY >= -0.1);
        if(rightYZero) {
            if(!pastRightYZero) {
                trebuchetArm.setPositionLock();
            }
            else{
                trebuchetArm.moveToTarget();
            }
        }
        pastRightYZero = rightYZero;
        telemetry.addData("rightYZero", rightYZero);*/
        
        if(rightY > 0.1){
            trebuchetArm.setVelocity(600 * rightY / 2);
        } else if(rightY < -0.1){
            trebuchetArm.setVelocity(1000 * rightY / 2);
        } else {
            trebuchetArm.setVelocity(1);
        }
        
        armPosition = trebuchetArm.getArmPosition();
        telemetry.addData("trebuchet arm position", "%,.3f", armPosition);
        boolean pressedX = gamepad1.x;
        if (pressedX && !pastX) {
            if(flywheelChanged) {
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
            //drive.toggleHalfSpeed();
            trebuchetArm.setZeroPosition();
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

        if(gamepad2.a) {
            tape.extendPower(1);
        }
        else if(gamepad2.b) {
            tape.extendPower(-1);
        }
        else {
            tape.extendPower(0);
        }
        
        if(Math.abs(gamepad2.left_stick_y) > 0.1) {
            tape.yawPower(gamepad2.left_stick_y);
        }
        else {
            tape.yawPower(0);
        }
        
        if(Math.abs(gamepad2.left_stick_x) > 0.1) {
            tape.pitchPower(gamepad2.left_stick_x);
        }
        else {
            tape.pitchPower(0);
        }
        
        telemetry.update();
    }

    public void stop() {
        //Turn Off Motors
        drive.stop();
        trebuchetArm.stop();
        toggles.stopIntake();
        toggles.stopFlywheel();
    }
}