package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name="TestArm")
public class TestArm extends OpMode{

    public DcMotor lowMotor;
    public DcMotor highMotor;
    String direction = "neutral";




    Thread threadLR = new Thread() {
        @Override
        public void run(){
            lowMotorRun();
        }
    };

    Thread threadHR = new Thread() {
        @Override
        public void run(){
            highMotorRun();
        }
    };

    Thread threadLRR = new Thread() {
        @Override
        public void run(){
            lowMotorRRun();
        }
    };

    Thread threadHRR = new Thread() {
        @Override
        public void run(){
            highMotorRRun();
        }
    };

    public void lowMotorRun() {
        lowMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lowMotor.setTargetPosition(5000);
        lowMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void highMotorRun() {
        highMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        highMotor.setTargetPosition(5000);
        highMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void lowMotorRRun() {
        lowMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lowMotor.setTargetPosition(-5000);
        lowMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void highMotorRRun() {
        highMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        highMotor.setTargetPosition(-5000);
        highMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    @Override
    public void init() {
        lowMotor = hardwareMap.get(DcMotor.class, "lowMotor");
        highMotor = hardwareMap.get(DcMotor.class, "highMotor");

        lowMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        highMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        lowMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        highMotor.setDirection(DcMotorSimple.Direction.FORWARD);
    }

    @Override
    public void loop() {

        /*
        if (gamepad1.a) {
            direction = "forward";
        } else if (gamepad1.b) {
            direction = "reverse";
        }

        if (direction == "forward") {
            lowMotorRun();
            highMotorRun();
            direction = "neutral";
        } else if (direction == "reverse"){
            lowMotorRRun();
            highMotorRRun();
        }

        */

        lowMotor.setPower(gamepad1.left_stick_y);
        highMotor.setPower(gamepad1.left_stick_y);

        lowMotor.setPower(gamepad1.right_stick_y);


    }
}

