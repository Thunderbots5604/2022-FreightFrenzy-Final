package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.hardware.Gyro;
import org.firstinspires.ftc.teamcode.hardware.TrebuchetArm;
import org.firstinspires.ftc.teamcode.hardware.mecanumdrive.MecanumDriveLibra;
import org.firstinspires.ftc.teamcode.math.linearalgebra.TwoDimensionalVector;

@TeleOp(name = "TeleOpTest", group = "TeleOp")
public class TeleOpTest extends OpMode {

    private Gyro gyro = new Gyro();

    //private Linears linears = new Linears(telemetry);
    private TrebuchetArm trebuchetArm;
    private Toggles toggles;

    private MecanumDriveLibra drive;
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

    /*
    private MecanumDrive drive;
    private Point direction;
    private double power;
    */

    @Override
    public void init() {
        drive = new MecanumDriveLibra(hardwareMap, "lmf", "rmf", "lmb", "rmb", true);
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
        /*
        //Check the multipliers for the motors
        //Reverse Toggle
        if(!pastX && gamepad1.x){
            drive.toggleReverse();
        }
        pastX = gamepad1.x;

        //Halfspeed Toggle
        if(!gamepad1.a && pastA){
            drive.toggleHalfSpeed();
        }
        pastA = gamepad1.a;
        drive.calculateMultiplier();

        //calculate and push the motor powers
        drive.resetPowerValues();

        direction = new Point(gamepad1.left_stick_x, gamepad1.left_stick_y);
        power = Math.sqrt(gamepad1.left_stick_x * gamepad1.left_stick_x +
                gamepad1.left_stick_y * gamepad1.left_stick_y);
        drive.linearMove(direction, power);
        drive.radialMove(gamepad1.right_stick_x);
        drive.updateMotorPowers();
        telemetry.addData("lmf power", drive.getFrontLeftMotorPower());
        telemetry.addData("rmf power", drive.getFrontRightMotorPower());
        telemetry.addData("lmb power", drive.getBackLeftMotorPower());
        telemetry.addData("rmb power", drive.getBackRightMotorPower());

        if(gamepad1.dpad_up && !pastDUp) {
            //drive.orient(0);
        }
        else if (gamepad1.dpad_right && !pastDRight) {
            //drive.orient(1);
        }
        else if (gamepad1.dpad_down && !pastDDown) {
            //drive.orient(2);
        }
        else if (gamepad1.dpad_left && !pastDLeft) {
            //drive.orient(3);
        }
        pastDUp = gamepad1.dpad_up;
        pastDRight = gamepad1.dpad_right;
        pastDDown = gamepad1.dpad_down;
        pastDLeft = gamepad1.dpad_left;
        */

        //run the teleop stuff and store the powers
        multiplier = 1;
        if (halfSpeed) {
            multiplier *= .5;
        }
        drive.powersToZero();
        drive.linearPowerCalculations(new TwoDimensionalVector(multiplier * gamepad1.left_stick_x, multiplier * gamepad1.left_stick_y));
        drive.pushPowers();
        //put the stuff into telemetry
        telemetry.addData("Status", "running");
        telemetry.addData("fl power", "%,.3f", drive.getFrontLeftPower());
        telemetry.addData("fr power", "%,.3f", drive.getFrontRightPower());
        telemetry.addData("bl power", "%,.3f", drive.getBackLeftPower());
        telemetry.addData("br power", "%,.3f", drive.getBackRightPower());



        if (gamepad1.right_trigger > 0) {
            trebuchetArm.setVelocity(gamepad1.right_trigger * 2200);
        } else if (gamepad1.left_trigger > 0) {
            trebuchetArm.setVelocity(-gamepad1.left_trigger * 2200);
        } else {
            trebuchetArm.stop();
        }
        armPosition = trebuchetArm.armPosition;
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
    /*public int quadrant() {
        double angle = gyro.getAngle() + Values.finalAngle;
        if (angle < 90) {
            return 2;
        }
        else if (angle < 180) {
            return 3;
        }
        else if (angle < 270) {
            return 4;
        }
        else {
            return 1;
        }
    }*/
}