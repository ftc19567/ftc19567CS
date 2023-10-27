package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class ClawTest extends OpMode {
    private Servo topClaw;
    private Servo bottomClaw;
    private Servo turnServo;

    private boolean turnServoB = false;

    private boolean topClawB = false;
    private boolean bottomClawB = false;

    @Override
    public void init() {
        topClaw = hardwareMap.get(Servo.class, "topClaw");
        bottomClaw = hardwareMap.get(Servo.class, "bottomClaw");
        turnServo = hardwareMap.get(Servo.class, "turnServo");
        topClaw.resetDeviceConfigurationForOpMode();
        bottomClaw.resetDeviceConfigurationForOpMode();
    }
    @Override
    public void loop() {

        /*

        if (gamepad1.y && !topClawB) {
            if(topClaw.getPosition() == 0.5) {
                topClaw.setPosition(0.15);
            } else {
                topClaw.setPosition(0.5);
            }
            topClawB = true;
        } else {
            topClawB = false;
        }

         */

        if (gamepad1.y) {
            // move to 0 degrees.
            topClaw.setPosition(topClaw.getPosition() + 0.1);
            telemetry.addData("Status", "open");

        }
        if (gamepad1.x) {
            // move to 90 degrees.
            //topClaw.setDirection(Servo.Direction.REVERSE);
            topClaw.setPosition(topClaw.getPosition() - 0.1);
            telemetry.addData("Status", "close");
        }
        //bottom
        if (gamepad1.b) {
            // move to 0 degrees.
            //bottomClaw.setDirection(Servo.Direction.REVERSE);
            bottomClaw.setPosition(0.15);
        }
        if (gamepad1.a) {
            // move to 90 degrees.
            //bottomClaw.setDirection(Servo.Direction.REVERSE);
            bottomClaw.setPosition(0.5);
        }

        //turnServo
        if (gamepad1.left_bumper && !turnServoB) {
            if(turnServo.getPosition()==0) {
                turnServo.setPosition(1);
            } else {
                turnServo.setPosition(0);
            }
            turnServoB = true;
        } else if (!gamepad1.left_bumper) {
            turnServoB = false;
        }

    }

}
