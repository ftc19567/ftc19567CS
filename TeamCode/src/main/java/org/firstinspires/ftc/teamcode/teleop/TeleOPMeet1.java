package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.mechanisms.Arm;
import org.firstinspires.ftc.teamcode.mechanisms.Intake;


@TeleOp
public class TeleOPMeet1 extends OpMode {


    private DcMotor frontLeftMotor;
    private DcMotor backLeftMotor;
    private DcMotor frontRightMotor;
    private DcMotor backRightMotor;
    private Servo turnServo;
    private Arm arm;
    private Intake intake;
    private boolean armUp = false;
    private boolean armDown = false;

    private boolean autoTurnServo = true;

    private double servoPosCalc = 1;


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

        telemetry.addData("Encoder :", arm.lowGetPosition());

        telemetry.update();

    }
    // Declare our motors
    // Make sure your ID's match your configuration

    @Override
    public void loop() {

        //Mecanum
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

        /*telemetry.addData("highMotor", arm.highGetPosition());
        telemetry.addData("lowMotor", arm.lowGetPosition());
         */
        telemetry.update();


        //ARM

        if (gamepad1.a && !armUp) {
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
            armUp = true;
            } else if (!gamepad1.a) {armUp = false;}

       /*
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

        */


        if (gamepad1.left_bumper) {
            Arm.negativeArmPower();

        }


        if (gamepad1.left_trigger > 0) {
            Arm.positiveArmPower();

        }



        //turning servo
        if (arm.lowGetPosition() > 1000) {
            servoPosCalc = 0.8 - (arm.lowGetPosition()/1600 * 0.35);
        }

        if (gamepad1.y) {autoTurnServo = true;}

        if (gamepad1.b) {autoTurnServo = false;}

        if (!autoTurnServo) {
            if (gamepad1.dpad_up) {
                turnServo.setPosition(turnServo.getPosition() - 0.1);
            }

            if (gamepad1.dpad_down) {
                turnServo.setPosition(turnServo.getPosition() + 0.1);
            }
        }  else {
            turnServo.setPosition(servoPosCalc);
        }




        //intake
        intake = new Intake(hardwareMap, telemetry);
        intake.rollingIntake(gamepad1.right_trigger);

        //ejects pixels
        intake.ejection((gamepad1.right_bumper));

    }
}