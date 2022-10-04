package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.math.MathUtilities;
import org.firstinspires.ftc.teamcode.math.Values;

public class TrebuchetArm {
    //motor!
    private DcMotorEx armMotor;
    private DcMotorEx armMotor2;
    private DcMotorEx armRotation;
    //raise and lower targets
    private int raiseTarget;
    private int lowerTarget;
    private int angleTarget;
    public double armAngle;
    public double armPosition;
    private int[] position;
    private int currentPosition;
    private int currentAngle;
    private int positionTolerance;
    private double idlePower;
    private double idlePowerRotate;
    private double movePower;
    private double movePowerRotate;
    private double rotatePower;
    private double downPower;
    private int zeroPosition;
    private double targetPosition;


    //constructor
    public TrebuchetArm(HardwareMap map, String armName, String arm2Name, /*String rotationName,*/ int raiseTarget, int lowerTarget, int[] position, int positionTolerance, double idlePower, double movePower, double downPower, double idlePowerRotate, double downPowerRotate, double movePowerRotate) {
        //motor set up
        armMotor = map.get(DcMotorEx.class, armName);
        armMotor2 = map.get(DcMotorEx.class, arm2Name);
        //armRotation = map.get(DcMotorEx.class, rotationName);

        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        armMotor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        armMotor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        armMotor2.setDirection(DcMotorSimple.Direction.REVERSE);


        //set the raising and lowering targets
        this.raiseTarget = raiseTarget;
        this.lowerTarget = lowerTarget;

        //set the positions
        this.position = position;
        this.positionTolerance = positionTolerance;
        this.idlePower = idlePower;
        this.movePower = movePower;
        this.downPower = downPower;
        this.idlePowerRotate = idlePowerRotate;
        this.movePowerRotate = movePowerRotate;
        this.rotatePower = rotatePower;

        currentPosition = 0;
        zeroPosition = 0;
    }
    public TrebuchetArm(HardwareMap hardwareMap) {
        armMotor = hardwareMap.get(DcMotorEx.class, "trebuchet");
        armMotor2 = hardwareMap.get(DcMotorEx.class, "trebuchet2");
        armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        armMotor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armMotor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armMotor2.setDirection(DcMotorSimple.Direction.REVERSE);
        raiseTarget = Values.raiseTarget;
        lowerTarget = Values.lowerTarget;
    }
    
    public void setPositionLock() {
        targetPosition = getArmPosition();
    }
    
    //PID stuff
    public static PIDCoefficients pidCoeffs = new PIDCoefficients(0, 0, 0);
    public PIDCoefficients pidGains = new PIDCoefficients(0, 0, 0); //PID gains which we will define later in the process

    ElapsedTime PIDTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS); //timer

    double lastError = 0;
    double integral = 0;
    //initializing our variables

    public void PID(double targetVelocity){
        PIDTimer.reset(); //resets the timer

        double currentVelocity = armMotor.getVelocity();
        double error = targetVelocity - currentVelocity; //pretty self explanatory--just finds the error

        double deltaError = error - lastError; //finds how the error changes from the previous cycle
        double derivative = deltaError / PIDTimer.time(); //deltaError/time gives the rate of change (sensitivity of the system)

        integral += error * PIDTimer.time();
        //continuously sums error accumulation to prevent steady-state error (friction, not enough p-gain to cause change)

        pidGains.p = error * pidCoeffs.p;
        //acts directly on the error; p-coefficient identifies how much to act upon it
        // p-coefficient (very low = not much effect; very high = lots of overshoot/oscillations)
        pidGains.i = integral * pidCoeffs.i;
        //multiplies integrated error by i-coefficient constant
        // i-coefficient (very high = fast reaction to steady-state error but lots of overshoot; very low = slow reaction to steady-state error)
        // for velocity, because friction isn't a big issue, only reason why you would need i would be for insufficient correction from p-gain
        pidGains.d = derivative * pidCoeffs.d;
        //multiplies derivative by d-coefficient
        // d-coefficient (very high = increased volatility; very low = too little effect on dampening system)

        armMotor.setVelocity(pidGains.p + pidGains.i + pidGains.d + targetVelocity);
        armMotor2.setVelocity(pidGains.p + pidGains.i + pidGains.d + targetVelocity);
        //adds up the P I D gains with the targetVelocity bias

        lastError = error;
        //makes our current error as our new last error for the next cycle
    }
    public void setZeroPosition() {
        zeroPosition = (armMotor.getCurrentPosition() - armMotor2.getCurrentPosition()) / 2;
    }

    //increase or decrease current position
    public void positionUp() {
        if (currentPosition < position.length - 1) {
            currentPosition++;
        }
    }

    public void positionDown() {
        if (currentPosition > 0) {
            currentPosition--;
        }
    }

    //move to the target position
    public boolean moveToTarget(double targetPos) {
        armMotor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        
        double actualPosition = (armMotor.getCurrentPosition() + armMotor2.getCurrentPosition()) / 2;
        if (MathUtilities.within(positionTolerance, actualPosition, targetPos)) {
            armMotor.setVelocity(1);
            armMotor2.setVelocity(1);
            return true;
        }
        else if (actualPosition < targetPos) {
            armMotor.setVelocity(500);
            armMotor2.setVelocity(500);
            return false;
        }
        else {
            armMotor.setVelocity(-500);
            armMotor2.setVelocity(-500);
            return false;
        }
    }

    //move to the target angle
    public void moveToTargetAngle(int targetAngle) {
        if(currentPosition == 0) {
            armRotation.setPower(0);
            return;
        }
        int actualAngle;
        targetAngle = position[currentPosition] + zeroPosition;
        actualAngle = (armRotation.getCurrentPosition());
        if (MathUtilities.within(positionTolerance, actualAngle, targetAngle)) {
            armRotation.setPower(idlePowerRotate);
        }
        else if (actualAngle < targetAngle) {
            armRotation.setPower(rotatePower);
        }
        else {
            armRotation.setPower(rotatePower);
        }
    }


    public void setCurrentPosition(int currentPosition) {
        this.currentPosition = currentPosition;
    }

    public void setArmAngle(int currentAngle){
        this.currentAngle = currentAngle;
    }

    //stop
    public void stop() {
        armMotor.setPower(0);
        armMotor2.setPower(0);
    }
    /*public void stopRotate(){
        armRotation.setPower(0);
    }*/

    //set velocity directly
    public void setVelocity(double velocity) {
        armMotor.setVelocity(velocity);
        armMotor2.setVelocity(velocity);
    }

    public void setPower(double power){
        armMotor2.setPower(power * 0.25);
        armMotor.setPower(power * 0.25);
    }
    //raise and lower
    public void raise() {
        setVelocity(-1200);
        armPosition = armMotor.getCurrentPosition();
    }
    public void lower() {
        setVelocity(1200);
        armPosition = armMotor.getCurrentPosition();
    }

    //autonomous raising and lowering to specific heights
    public void autoRaise() {
        while (armMotor.getCurrentPosition() > raiseTarget) {
            setVelocity(-2400);
            armPosition = armMotor.getCurrentPosition();
        }
        stop();
    }
    public void autoLower() {
        while(armMotor.getCurrentPosition() < lowerTarget) {
            setVelocity(2400);
            armPosition = armMotor.getCurrentPosition();
        }
        stop();
    }

    /*public void autoAngle(int angleTarget){
        this.angleTarget = angleTarget;
        if (armRotation.getCurrentPosition() < angleTarget){
            setVelocity(2400);
            armAngle = armRotation.getCurrentPosition();
        }
        else if(armRotation.getCurrentPosition() > angleTarget){
            setVelocity(-2400);
            armAngle = armRotation.getCurrentPosition();
        } else {stopRotate();}


    }*/


    public double getArmPosition() {
        return armMotor.getCurrentPosition() + armMotor2.getCurrentPosition() / 2;
    }

    public int getCurrentPosition() {
        return currentPosition;
    }
    
    
    
    
}
