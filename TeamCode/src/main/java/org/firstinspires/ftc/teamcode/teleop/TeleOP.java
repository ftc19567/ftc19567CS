package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.mechanisms.Arm;
import org.firstinspires.ftc.teamcode.mechanisms.Intake;


@TeleOp
public class TeleOP extends OpMode {


    private DcMotor frontLeftMotor;
    private DcMotor backLeftMotor;
    private DcMotor frontRightMotor;
    private DcMotor backRightMotor;
    private Servo turnServo;
    private Arm arm;
    private Intake intake;
    private boolean armUp = false;
    private boolean armDown = false;

    private boolean turnServoB = false;

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

        turnServo = hardwareMap.get(Servo.class, "turnServo");

        //TURNING SERVO
        turnServo.setPosition(0.8);

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
            if (arm.highGetPosition() <= 1000) {
                turnServo.setPosition(1);
                try {
                    Thread.sleep(500);
                } catch (InterruptedException e) {
                    throw new RuntimeException(e);
                }
                arm.setPosition(1, 1500);
                try {
                    Thread.sleep(1600);
                } catch (InterruptedException e) {
                    throw new RuntimeException(e);
                }
                turnServo.setPosition(0.4);
            } else {
                turnServo.setPosition(1);
                try {
                    Thread.sleep(500);
                } catch (InterruptedException e) {
                    throw new RuntimeException(e);
                }
                arm.setPosition(1, -16);
                try {
                    Thread.sleep(2000);
                } catch (InterruptedException e) {
                    throw new RuntimeException(e);
                }
                turnServo.setPosition(0.8);
            }
            liftArm = true;
        } else if (!gamepad1.right_bumper) {
            liftArm = false;
        }


        if (gamepad1.a && !armDown) {
            Arm.negativeArmPower();
            armDown = true;
        } else if (!gamepad1.a) {
            armDown = false;
        }

        if (gamepad1.y && !armUp) {
            Arm.positiveArmPower();
            armUp = false;
        } else if (!gamepad1.y) {
            armUp = false;
        }

/*
        //turning servo
        if (gamepad1.left_bumper && !turnServoB) {
            if (turnServo.getPosition() == 0.9) {
                turnServo.setPosition(1);
            } else if (turnServo.getPosition() == 1){
                turnServo.setPosition(0.5);
            } else {
                turnServo.setPosition(0.9);
            }
            turnServoB = true;
        } else if (!gamepad1.left_bumper) {
            turnServoB = false;
        }

 */

        //intake
        intake = new Intake(hardwareMap, telemetry);

        intake.rollingIntake(gamepad1.right_trigger, gamepad1.left_trigger);

    }
}