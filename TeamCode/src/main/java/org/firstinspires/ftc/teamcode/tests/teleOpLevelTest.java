package org.firstinspires.ftc.teamcode.tests;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.mechanisms.Arm;

@Config
@TeleOp
public class teleOpLevelTest extends OpMode {
    Arm arm;
    Servo turnServo;

    private boolean liftArmUp = false;
    private boolean liftArmDown = false;

    DcMotor frontLeftMotor, frontRightMotor, backLeftMotor, backRightMotor;

    private boolean timerUpB = false;

    private boolean timerUpFB = false;

    private boolean timerDownB = false;



    ElapsedTime timerUp = new ElapsedTime();
    ElapsedTime timerUpFinsh = new ElapsedTime();
    ElapsedTime timerDown = new ElapsedTime();
    ElapsedTime timerDownFinish = new ElapsedTime();

    public void init() {

        arm = new Arm(hardwareMap, telemetry);
        turnServo = hardwareMap.get(Servo.class, "turnServo");

        turnServo.setPosition(0.67);
        Arm.setPosition(1, 0);

        frontLeftMotor = hardwareMap.get(DcMotor.class, "frontLeftMotor");
        backLeftMotor = hardwareMap.get(DcMotor.class, "backLeftMotor");
        frontRightMotor = hardwareMap.get(DcMotor.class, "frontRightMotor");
        backRightMotor = hardwareMap.get(DcMotor.class, "backRightMotor");

        // Reverse the right side motors. This may be wrong for your setup.
        // If your robot moves backwards when commanded to go forwards,
        // reverse the left side instead.
        // See the note about this earlier on this page.
        frontLeftMotor.setDirection(DcMotor.Direction.REVERSE);

    }
    public void loop() {

        double y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
        double x = gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
        double rx = gamepad1.right_stick_x * 0.5;

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

        /*
        turnServo.setPosition(0.85);
                try {
                    Thread.sleep(100);
                } catch (InterruptedException e) {
                    throw new RuntimeException(e);
                }
                arm.setPosition(0.6, 1750);
                try {
                    Thread.sleep(700);
                } catch (InterruptedException e) {
                    throw new RuntimeException(e);
                }
                turnServo.setPosition(0.18);

         */

        /*
        turnServo.setPosition(1);
                try {
                    Thread.sleep(500);
                } catch (InterruptedException e) {
                    throw new RuntimeException(e);
                }
                arm.setPosition(1, 5);
                try {
                    Thread.sleep(1000);
                } catch (InterruptedException e) {
                    throw new RuntimeException(e);
                }
                turnServo.setPosition(0.67);
         */

        if (gamepad2.a && !liftArmUp && timerUpB == false && timerUpFB == false) {
            if (arm.highGetPosition() <= 1000) {
                turnServo.setPosition(0.85);
                liftArmUp = true;
                timerUp.reset();
                timerUpB = true;

            } }else if (!gamepad2.a) {liftArmUp = false;}

        if (timerUp.milliseconds() >= 100 && timerUpB == true) {
            arm.setPosition(0.6, 1750);
            timerUpB = false;
            timerUpFinsh.reset();
            timerUpFB = true;
        }
        if (timerUpFinsh.milliseconds() >= 700 && timerUpFB == true) {
            turnServo.setPosition(0.18);
            timerUpFB = false;
        }



        //LiftArmDown

        if (gamepad2.b && !liftArmDown && timerDownB == false) {
            if (arm.highGetPosition() > 1000) {
                turnServo.setPosition(1);
                arm.setPosition(1, 5);
                liftArmDown = true;
                timerDownB = true;
                timerDown.reset();
            }} else if (!gamepad2.b) {liftArmDown = false;}

        if (timerDown.milliseconds() >= 1000  && timerDownB == true) {
            turnServo.setPosition(0.67);
            timerDownB = false;
        }



        telemetry.addData("liftArmUpB", liftArmUp);
        telemetry.addData("liftArmDownB", liftArmDown);
        telemetry.addData("armPos", arm.lowGetPosition());





    }

}
