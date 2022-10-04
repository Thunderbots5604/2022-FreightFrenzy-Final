package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.hardware.MecanumDrive;
import org.firstinspires.ftc.teamcode.math.Point;

@TeleOp(name = "PushBot", group = "TeleOp")
public class PushBot extends OpMode {

    private DcMotorEx intake;

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
    private boolean engageRing = false;
    private boolean pastDUp = false;
    private boolean pastDRight = false;
    private boolean pastDDown = false;
    private boolean pastDLeft = false;
    private boolean past2X = false;
    private boolean allOff = false;
    private boolean scoopChanged = false;
    private boolean flywheelChanged = false;
    private boolean intakeChanged = false;

    private MecanumDrive drive;
    private Point direction;
    private double power;
    private TrebuchetArmDistance trebuchetArm;

    @Override
    public void init() {
        //Get Hardware Map
        drive = new MecanumDrive(hardwareMap);


        drive.resetPowerValues();

        telemetry.addData("lmf power", drive.getFrontLeftMotorPower());
        telemetry.addData("rmf power", drive.getFrontRightMotorPower());
        telemetry.addData("lmb power", drive.getBackLeftMotorPower());
        telemetry.addData("rmb power", drive.getBackRightMotorPower());

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        intake = hardwareMap.get(DcMotorEx.class, "intake");




    }

    public void start() {
    }

    @Override
    public void loop() {
        leftTrigger = gamepad2.left_trigger;
        rightTrigger = gamepad2.right_trigger;

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

        //last minute add in intake
        boolean pressedY = gamepad1.y;
        if (pressedY && !pastY) {
            if (intakeChanged) {
                intake.setVelocity(100);
                intakeChanged = false;
            }
            else {
                intake.setVelocity(0);
                intakeChanged = true;
            }
        }
        pastY = pressedY;


        trebuchetArm.liftArm(0.1);

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

    }

    public void stop() {
        //Turn Off Motors
        drive.stopMotors();
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