package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;


public class Turret {
    private DcMotor turret;
    private double autoSpeed = 0.7;

    public Turret(HardwareMap map, String turretName) {
        turret = map.get(DcMotor.class, turretName);
        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void setPower(double power) {
        turret.setPower(power);
    }

    public boolean turnForward() {
        if(Math.abs(turret.getCurrentPosition()) > 50) {
            if(turret.getCurrentPosition() < 0) {
                setPower(autoSpeed);
            }
            else {
                setPower(-autoSpeed);
            }
        }
        else {
            setPower(0);
            return true;
        }
        return false;
    }

    public boolean turnLeft() {
        if(Math.abs(turret.getCurrentPosition() - 11000) > 50) {
            if(turret.getCurrentPosition() < 11000) {
                setPower(autoSpeed);
            }
            else {
                setPower(-autoSpeed);
            }
        }
        else {
            setPower(0);
            return true;
        }
        return false;
    }

    public boolean turnRight() {
        if(Math.abs(turret.getCurrentPosition() - -11000) > 50) {
            if(turret.getCurrentPosition() < -11000) {
                setPower(autoSpeed);
            }
            else {
                setPower(-autoSpeed);
            }
        }
        else {
            setPower(0);
            return true;
        }
        return false;
    }

    //true is clockwise, false is counterclockwise
    public boolean turnBackward(boolean direction) {
        if (direction) {
            if(Math.abs(turret.getCurrentPosition() - -22000) > 50) {
                if(turret.getCurrentPosition() < -22000) {
                    setPower(autoSpeed);
                }
                else {
                    setPower(-autoSpeed);
                }
            }
            else {
                setPower(0);
                return true;
            }
            return false;
        } else {
            if(Math.abs(turret.getCurrentPosition() - 22000) > 50) {
                if(turret.getCurrentPosition() < 22000) {
                    setPower(autoSpeed);
                }
                else {
                    setPower(-autoSpeed);
                }
            }
            else {
                setPower(0);
                return true;
            }
            return false;
        }
    }
}