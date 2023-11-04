package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.mechanisms.Arm;


@TeleOp
public class TeleOP extends OpMode {


    private DcMotor frontLeftMotor;
    private DcMotor backLeftMotor;
    private DcMotor frontRightMotor;
    private DcMotor backRightMotor;
    private Servo topClaw;
    private Servo bottomClaw;
    private Servo turnServo;
    private Arm arm;
    private boolean turnServoB = false;
    private boolean negPowerB = false;
    private boolean posPowerB = false;

    private boolean yclawB = false;
    private boolean xclawB = false;

    private boolean liftArm = false;



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
        arm = new Arm(hardwareMap, telemetry);
        /*lowMotor = hardwareMap.get(DcMotor.class, "lowMotor");
        highMotor = hardwareMap.get(DcMotor.class, "highMotor");

        lowMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        highMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        lowMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        highMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        lowMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        highMotor.setDirection(DcMotorSimple.Direction.FORWARD);
         */

        //CLAW
        topClaw = hardwareMap.get(Servo.class, "topClaw");
        bottomClaw = hardwareMap.get(Servo.class, "bottomClaw");
        turnServo = hardwareMap.get(Servo.class, "turnServo");
        topClaw.resetDeviceConfigurationForOpMode();
        bottomClaw.resetDeviceConfigurationForOpMode();

        //TURNING SERVO
        turnServo.setPosition(0);

        //Claws
        topClaw.setPosition(0);
        bottomClaw.setPosition(0);


    }
    // Declare our motors
    // Make sure your ID's match your configuration

    @Override
    public void loop() {

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

        /*telemetry.addData("highMotor", arm.highGetPosition());
        telemetry.addData("lowMotor", arm.lowGetPosition());
         */
        telemetry.update();


        //ARM
        if (gamepad1.right_bumper && !liftArm) {
            if(arm.highGetPosition()<=200) {
                arm.setPosition(1, 1280);
            } else {
                arm.setPosition(1, 0);
            }
            liftArm = true;
        } else if (!gamepad1.right_bumper) {
            liftArm = false;
        }


        if (gamepad1.b) {
            Arm.negativeArmPower();
        }

        if (gamepad1.a) {
            Arm.positiveArmPower();
        }

        //CLAWS
        //top

        if (gamepad1.y && !yclawB) {
            if(topClaw.getPosition()==0) {
                topClaw.setPosition(0.12);
            } else {
                topClaw.setPosition(0);
            }
            yclawB = true;
        } else if (!gamepad1.y) {
            yclawB = false;
        }

        if (gamepad1.x && !xclawB) {
            if(bottomClaw.getPosition()==0) {
                bottomClaw.setPosition(0.175);
            } else {
                bottomClaw.setPosition(0);
            }
            xclawB = true;
        } else if (!gamepad1.x) {
            xclawB = false;
        }


        //turning servo
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