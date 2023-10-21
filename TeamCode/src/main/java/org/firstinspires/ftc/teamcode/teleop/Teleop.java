package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp
public class Teleop extends OpMode {



    DcMotor frontLeftMotor;
    DcMotor backLeftMotor;
    DcMotor frontRightMotor;
    DcMotor backRightMotor;

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

    @Override
    public void init() {
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
    }
    // Declare our motors
    // Make sure your ID's match your configuration

    @Override
    public void loop() {
        MecanumDrive(1, 0);

    }
}

