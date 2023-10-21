package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.util.Mechanisms;
import org.firstinspires.ftc.teamcode.util.Utility_Constants;


@TeleOp
public class TeleOP extends OpMode {

    private DcMotor frontLeftMotor;
    private DcMotor backLeftMotor;
    private DcMotor frontRightMotor;
    private DcMotor backRightMotor;
    private Servo topClaw;
    private Servo bottomClaw;
    private Servo turnServo;
    public DcMotor lowMotor;
    public DcMotor highMotor;
    int num = 0;
    int num1 = 0;
    int num2 = 1;

    double frontLeftPower;
    double frontRightPower;
    double backLeftPower;
    double backRightPower;


    public void MecanumDrive(double powerMult, double deadZone) {
        double x = gamepad1.left_stick_y;
        double y = gamepad1.left_stick_x;
        double pivot = gamepad1.right_stick_x;

        if (Math.abs(x) < Math.abs(deadZone)) {
            x = 0;
        }
        if (Math.abs(y) < Math.abs(deadZone)) {
            y = 0;
        }

        double theta = Math.atan2(y, x);
        double power = Math.hypot(x,y);

        double mSin = Math.sin(theta - Math.PI/4);
        double mCos = Math.cos(theta - Math.PI/4);
        double mMax = Math.max(Math.abs(mSin), Math.abs(mCos));

        frontLeftPower = mSin * power/mMax + pivot;
        frontRightPower = mCos * power/mMax - pivot;
        backLeftPower = mSin * power/mMax + pivot;
        backRightPower = mCos * power/mMax - pivot;

        if (power + Math.abs(pivot) > 1) {
            frontLeftPower /= power + Math.abs(pivot);
            frontRightPower /= power + Math.abs(pivot);
            backLeftPower /= power + Math.abs(pivot);
            backRightPower /= power + Math.abs(pivot);
        }

        //frontLeftPowerMult
        if (frontLeftPower * powerMult > 1) {
            frontLeftPower = 1;
        }
        if (frontLeftPower * powerMult < -1) {
            frontLeftPower = -1;
        }
        //frontRightPowerMult
        if (frontRightPower * powerMult > 1) {
            frontRightPower = 1;
        }
        if (frontRightPower * powerMult < -1) {
            frontRightPower = -1;
        }
        //backLeftPowerMult
        if (backLeftPower * powerMult > 1) {
            backLeftPower = 1;
        }
        if (backLeftPower * powerMult < -1) {
            backLeftPower = -1;
        }
        //backRightPowerMult
        if (backRightPower * powerMult > 1) {
            backRightPower = 1;
        }
        if (backRightPower * powerMult < -1) {
            backRightPower = -1;
        }

        frontLeftMotor.setPower(frontLeftPower);
        frontRightMotor.setPower(frontRightPower);
        backLeftMotor.setPower(backLeftPower);
        backRightMotor.setPower(backRightPower);

    }

    public void negativeArmPower () {
        lowMotor.setTargetPosition(100);
        lowMotor.setPower(-gamepad1.left_trigger);
        highMotor.setTargetPosition(100);
        highMotor.setPower(-gamepad1.left_trigger);
    }

    public void positiveArmPower () {
        lowMotor.setTargetPosition(200);
        lowMotor.setPower(gamepad1.left_trigger);
        highMotor.setTargetPosition(200);
        highMotor.setPower(gamepad1.left_trigger);
    }

    @Override
    public void init() {
        //MECANUM
        frontLeftMotor = hardwareMap.get(DcMotor.class, "frontLeftMotor");
        backLeftMotor = hardwareMap.get(DcMotor.class, "backLeftMotor");
        frontRightMotor = hardwareMap.get(DcMotor.class, "frontRightMotor");
        backRightMotor = hardwareMap.get(DcMotor.class, "backRightMotor");

        // Reverse the right side motors. This may be wrong for your setup.
        // If your robot moves backwards when commanded to go forwards,
        // reverse the left side instead.
        // See the note about this earlier on this page.
        frontRightMotor.setDirection(DcMotor.Direction.REVERSE);
        backRightMotor.setDirection(DcMotor.Direction.REVERSE);

        //ARM
        lowMotor = hardwareMap.get(DcMotor.class, "lowMotor");
        highMotor = hardwareMap.get(DcMotor.class, "highMotor");

        lowMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        highMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        lowMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        highMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        lowMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        highMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        lowMotor.setTargetPosition(0);
        highMotor.setTargetPosition(0);

        lowMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        highMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        //CLAW
        topClaw = hardwareMap.get(Servo.class, "topClaw");
        bottomClaw = hardwareMap.get(Servo.class, "bottomClaw");
        turnServo = hardwareMap.get(Servo.class, "turnServo");
    }
    // Declare our motors
    // Make sure your ID's match your configuration

    @Override
    public void loop() {
        /* OLD MECANUM
        //Mecanum
        double y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
        double x = gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
        double rx = gamepad1.right_stick_x;

        // Denominator is the largest motor power (absolute value) or 1
        // This ensures all the powers maintain the same ratio,
        // but only if at least one is out of the range [-1, 1]
        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        double frontLeftPower = (y + x + rx) / denominator;
        double backLeftPower = (y - x + rx) / denominator;
        double frontRightPower = (y - x - rx) / denominator;
        double backRightPower = (y + x - rx) / denominator;

        frontLeftMotor.setPower(frontLeftPower);
        backLeftMotor.setPower(backLeftPower);
        frontRightMotor.setPower(frontRightPower);
        backRightMotor.setPower(backRightPower);
        */

        MecanumDrive(1, 0.1);


        //ARM
        //ARM UP
        /*if(gamepad1.left_bumper) {
            lowMotor.setPower(1);
            highMotor.setPower(1);
        }
        //SLOW UP
        if(gamepad1.right_bumper) {
            lowMotor.setPower(1);
        }
        //ARM DOWN

         */

        /*
        lowMotor.setPower(gamepad1.left_trigger);
        highMotor.setPower(gamepad1.left_trigger);

        lowMotor.setPower(-(gamepad1.right_trigger));
        highMotor.setPower(-(gamepad1.right_trigger));

         */
        negativeArmPower();
        positiveArmPower();
        //CLAWS
        //top

        if (num % 2 == 1) {
            if (gamepad1.y) {
                // move to 0 degrees.
                topClaw.setPosition(0);
                num += 1;
            }
        } else {
            if (gamepad1.y) {
                // move to 90 degrees.
                topClaw.setPosition(0.5);
                num += 1;
            }
        }
        //bottom
        if (num1 % 2 == 1) {
            if (gamepad1.x) {
                // move to 0 degrees.
                bottomClaw.setPosition(0);
                num1 += 1;
            }
        } else {
            if (gamepad1.x) {
                // move to 90 degrees.
                bottomClaw.setPosition(0.5);
                num1 += 1;
            }
        }
        //turning servo
        switch (num2) {
            case 1:
                if (gamepad1.b) {
                    // move to 0 degrees.
                    turnServo.setPosition(0);
                    num2 += 1;
                }
                break;
            case 2:
                if (gamepad1.b) {
                    // move to 90 degrees.
                    turnServo.setPosition(0.5);
                    num2 += 1;
                }
                break;
            case 3:
                if (gamepad1.b) {
                    // move to 180 degrees.
                    turnServo.setPosition(1);
                    num2 = 0;
                }
                break;
        }

    }
}





