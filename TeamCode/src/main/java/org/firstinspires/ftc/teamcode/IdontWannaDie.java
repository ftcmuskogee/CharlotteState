package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.util.ElapsedTime;


import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class IdontWannaDie {
    boolean y = false;
    // names motors and sets motors to type null
    public DcMotor Frontright = null;
    public DcMotor Backright = null;
    public DcMotor Backleft = null;
    public DcMotor Frontleft = null;
    public DcMotor LinAct = null;
    public Servo ClawL = null;
    public Servo ClawR = null;
    // wrist
    public Servo Wrist = null;
    // linear actuator angle change
    //may need changing
    public CRServo LinAngle = null;
    // arm angle
    public DcMotor ArmAngle = null;
    public DcMotor Armextend = null;

    //plane launch
    public Servo Plane = null;

    // sets hardware map to null and names it
    HardwareMap IdontWannaDie = null;
    // creates runtime variable
    public ElapsedTime runtime = new ElapsedTime();

    public void init(HardwareMap hmap) {
        //sets up names for configuration
        IdontWannaDie = hmap;

        Frontright = hmap.get(DcMotor.class, "RF");
        Backright = hmap.get(DcMotor.class, "RB");
        Backleft = hmap.get(DcMotor.class, "LB");
        Frontleft = hmap.get(DcMotor.class, "LF");
        LinAct = hmap.get(DcMotor.class, "ACT ");
        ClawL = hmap.get(Servo.class, "CL");
        ClawR = hmap.get(Servo.class, "CR");
        Armextend = hmap.get(DcMotor.class, "ARM");
        Wrist = hmap.get(Servo.class, "W");
        LinAngle = hmap.get(CRServo.class, "LA");
        ArmAngle = hmap.get(DcMotor.class, "AA");
        Plane = hmap.get(Servo.class, "P");

        ClawL.setPosition(0);
        ClawR.setPosition(0.5);

        Frontleft.setDirection(DcMotor.Direction.REVERSE);
        Backleft.setDirection(DcMotor.Direction.REVERSE);


        // sets the lifts zeropowerbehavior to brake
        Frontright.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Backright.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Frontleft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Backleft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        Armextend.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Armextend.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Armextend.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        ArmAngle.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        ArmAngle.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        ArmAngle.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        LinAct.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LinAct.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LinAct.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//close


    }

    // function for driving forward
    //runs motors forward at 60% power
    public void Forward(double seconds) {
        runtime.reset();
        while (runtime.milliseconds() < (seconds * 1000)) {
            Frontleft.setPower(0.6);
            Frontright.setPower(0.6);
            Backleft.setPower(0.6);
            Backright.setPower(0.6);
        }
    }

    //function for driving backward
    //runs motors backward at 60% power
    public void Backward(double seconds) {
        runtime.reset();
        while (runtime.milliseconds() < (seconds * 1000)) {
            Frontleft.setPower(-0.6);
            Frontright.setPower(-0.6);
            Backleft.setPower(-0.6);
            Backright.setPower(-0.6);
        }
    }

    // function for turning left
    //runs left motors backward at 60% power
    //runs right motors forward at 60% power
    public void Left(double seconds) {
        runtime.reset();
        while (runtime.milliseconds() < (seconds * 1000)) {
            Frontleft.setPower(-0.6);
            Frontright.setPower(0.6);
            Backleft.setPower(-0.6);
            Backright.setPower(0.6);
        }
    }

    // function for turning right
    //runs right motors backward at 60% power
    //runs left motors forward at 60% power
    public void Right(double seconds) {
        runtime.reset();

        while (runtime.milliseconds() < (seconds * 1000)) {
            Frontleft.setPower(0.6);
            Frontright.setPower(-0.6);
            Backleft.setPower(0.6);
            Backright.setPower(-0.6);

        }
    }

    // function for strafing right
    //runs frontleft motor forward at 60% power
    //runs frontright motor backward at 60% power
    //runs backleft motor backward at 60% power
    //runs backright motor forward at 60% power
    public void RightStrafe(double seconds) {
        runtime.reset();

        while (runtime.milliseconds() < (seconds * 1000)) {
            Frontleft.setPower(0.6);
            Frontright.setPower(-0.6);
            Backleft.setPower(-0.6);
            Backright.setPower(0.6);

        }
    }

    // function for strafing left
    //runs frontleft motor backward at 60% power
    //runs frontright motor forward at 60% power
    //runs backleft motor forward at 60% power
    //runs backright motor backward at 60% power
    public void LeftStrafe(double seconds) {
        runtime.reset();
        while (runtime.milliseconds() < (seconds * 1000)) {
            Frontleft.setPower(-0.6);
            Frontright.setPower(0.6);
            Backleft.setPower(0.6);
            Backright.setPower(-0.6);
        }
    }

    // function for turning off motors
    public void Off() {
        Frontright.setPower(0);
        Frontleft.setPower(0);
        Backleft.setPower(0);
        Backright.setPower(0);
    }

    public void Aoff() {
        Armextend.setPower(0);
        ArmAngle.setPower(0);
    }
    //close right side claw
    public void CR(double position) {
        ClawR.setPosition(position);
    }

    //close left side claw
    public void CL(double position) {
        ClawL.setPosition(position);
    }

    //claw wrist
    public void W(double position) {
        Wrist.setPosition(position);
    }

    // plane launch
    public void P(double position) {
        Plane.setPosition(position);
    }

    public void LA(double position) {
        LinAngle.setPower(position);
    }

    //function for moving arm 1 into bot
    //runs arm 1 motor in at 100% power
    public void IN(double seconds) {
        double time = (seconds * 1000) + runtime.milliseconds();
        while (time > runtime.milliseconds()) {
            Armextend.setPower(.2);
        }

    }

    //function for moving arm 1 out from bot
    // runs arm 1 motor out at 100% power
    public void OUT(double seconds) {
        double time = (seconds * 1000) + runtime.milliseconds();
        while (time > runtime.milliseconds()) {
            Armextend.setPower(.2);
        }
    }

    public void UP(double seconds) {
        double time = (seconds * 1000) + runtime.milliseconds();
        while (time > runtime.milliseconds()) {
            ArmAngle.setPower(.01);
        }
    }

    public void DOWN(double seconds) {
        double time = (seconds * 1000) + runtime.milliseconds();
        while (time > runtime.milliseconds()) {
            ArmAngle.setPower(-.2);
        }
    }

}