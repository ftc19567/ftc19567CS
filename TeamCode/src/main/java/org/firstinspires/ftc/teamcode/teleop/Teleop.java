package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.util.Mechanisms;
import org.firstinspires.ftc.teamcode.util.Preset_State;
import org.firstinspires.ftc.teamcode.util.Utility_Constants;
import org.firstinspires.ftc.teamcode.util.TeleOp_State;

//From Previous
//import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

//Custom enums and classes
//import org.firstinspires.ftc.Team19567.drive.MecanumDriveCancelable;



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

<<<<<<< Updated upstream
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
=======
    private boolean negPowerB = false;
    private boolean posPowerB = false;

    //From Previous
    //Variables
    private double armPos = 0;
    private double armPower = 1.0;
    private double releaseServoPos = 0.75;
    private double acc = Utility_Constants.MAX_SENSITIVITY;
    private double carouselPower = 0.0;
    private boolean isSlowmode = false;
    private boolean isCarouselEngaged = false;
    private boolean isIntaked = false;
    private boolean redAlliance = true;

    //Hardware
    private DcMotor leftDCFront = null;
    private DcMotor rightDCFront = null;
    private DcMotor leftDCBack = null;
    private DcMotor rightDCBack = null;
    private DcMotorEx armDC = null;
    private Servo releaseServo = null;
    private Servo balanceServo = null;
    private TouchSensor limitSwitch = null;
    private DistanceSensor distanceSensor = null;
    private RevBlinkinLedDriver blinkin = null;
    private AnalogInput forceSensor = null;

    private Mechanisms mechanisms = null;

    private Preset_State presetState = Preset_State.NO_PRESET;
    private TeleOp_State currentState = TeleOp_State.DRIVER_CONTROL;

>>>>>>> Stashed changes

    public void negativeArmPower () {
        lowMotor.setTargetPosition(lowMotor.getCurrentPosition() - 10);
        highMotor.setTargetPosition(highMotor.getCurrentPosition() - 10);
        lowMotor.setPower(-0.4);
        highMotor.setPower(-0.4);
    }

    public void positiveArmPower () {
        lowMotor.setTargetPosition(lowMotor.getCurrentPosition() + 10);
        highMotor.setTargetPosition(highMotor.getCurrentPosition() + 10);
        lowMotor.setPower(0.4);
        highMotor.setPower(0.4);
    }

<<<<<<< Updated upstream
=======


>>>>>>> Stashed changes
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

<<<<<<< Updated upstream
        lowMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        highMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        lowMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        highMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        lowMotor.setTargetPosition(0);
        highMotor.setTargetPosition(0);
=======
        lowMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        highMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        lowMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        highMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
>>>>>>> Stashed changes

        lowMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        highMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        //CLAW
        topClaw = hardwareMap.get(Servo.class, "topClaw");
        bottomClaw = hardwareMap.get(Servo.class, "bottomClaw");
        turnServo = hardwareMap.get(Servo.class, "turnServo");
<<<<<<< Updated upstream
=======
        topClaw.resetDeviceConfigurationForOpMode();
        bottomClaw.resetDeviceConfigurationForOpMode();

        mechanisms = new Mechanisms(hardwareMap,telemetry);
>>>>>>> Stashed changes
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
        /* if(gamepad1.left_bumper) {
            lowMotor.setPower(1);
            highMotor.setPower(1);
        }
        //SLOW UP
        if(gamepad1.right_bumper) {
            lowMotor.setPower(1);
        }

         */
<<<<<<< Updated upstream

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
=======
        //ARM DOWN

        /* if (gamepad2.left_bumper && !negPowerB) {
            negativeArmPower();
            negPowerB = true;
        } else if (!gamepad2.left_bumper) {
            negPowerB = false;
        }

        if (gamepad2.right_bumper && !posPowerB) {
            positiveArmPower();
            posPowerB = true;
        } else if (!gamepad2.right_bumper) {
            posPowerB = false;
        }

         */

        //CLAWS
        //top

        /*
        if (gamepad1.y) {
            // move to 0 degrees.
            topClaw.setPosition(0);
            num += 1;
        }
        if (gamepad1.x) {
            // move to 90 degrees.
            topClaw.setPosition(0.5);
            num += 1;
>>>>>>> Stashed changes
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
<<<<<<< Updated upstream
        //turning servo
=======
>>>>>>> Stashed changes
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

        //From Previous
        if(gamepad1.left_trigger > 0 || gamepad2.left_trigger > 0) armPos = Range.clip(armPos+gamepad1.left_trigger*8,0,Utility_Constants.MAX_POS);
        else if(gamepad1.left_trigger > 0 || gamepad2.left_trigger > 0) armPos = Range.clip(armPos+gamepad2.left_trigger*8,0,Utility_Constants.MAX_POS);
        if(gamepad1.left_bumper || gamepad2.left_bumper) armPos = Range.clip(armPos-8,0,Utility_Constants.MAX_POS);
        if(gamepad1.x || gamepad2.x) {
            presetState = Preset_State.ALLIANCE_FIRST;
        }
        else if(gamepad2.y) {
            presetState = Preset_State.ALLIANCE_SECOND;
        }
        else if(gamepad1.a || gamepad2.a) {
            presetState = Preset_State.ALLIANCE_THIRD;
        }
        else if(gamepad1.b || gamepad2.b) {
            presetState = Preset_State.GOING_DOWN;
        }

        if(presetState != Preset_State.NO_PRESET) {
            mechanisms.moveIntake(0.4);
        }
        //PRESET HANDLING
        switch(presetState) {
            case ALLIANCE_FIRST: {
                armPower = Utility_Constants.FIRST_LEVEL_POWER;
                armPos = Utility_Constants.FIRST_LEVEL_POS;
                if(armDC.getCurrentPosition() >= Utility_Constants.FIRST_LEVEL_POS-5) {
                    presetState = Preset_State.NO_PRESET;
                }
                break;
            }
            case ALLIANCE_SECOND: {
                armPower = Utility_Constants.SECOND_LEVEL_POWER;
                armPos = Utility_Constants.SECOND_LEVEL_POS;
                if(armDC.getCurrentPosition() >= Utility_Constants.SECOND_LEVEL_POS-5) {
                    presetState = Preset_State.NO_PRESET;
                }
                break;
            }
            case ALLIANCE_THIRD: {
                armPower = Utility_Constants.THIRD_LEVEL_POWER;
                armPos = Utility_Constants.THIRD_LEVEL_POS;
                if(armDC.getCurrentPosition() >= Utility_Constants.THIRD_LEVEL_POS-5) {
                    presetState = Preset_State.NO_PRESET;
                }
                break;
            }
            case GOING_DOWN: {
                releaseServoPos = Utility_Constants.RELEASE_SERVO_DEFAULT;
                mechanisms.moveIntake(-0.4);
                armPower = Utility_Constants.GOING_DOWN_POWER;
                armPos = 0;
                if(armDC.getCurrentPosition() <= 5) {
                    presetState = Preset_State.NO_PRESET;
                }
                break;
            }
            default: {
                armPower = 1.0;
                break;
            }
        }

        //SERVOS
        /*if(gamepad1.dpad_down) releaseServoPos = Range.clip(releaseServoPos-Utility_Constants.SERVO_SENSITIVITY,releaseServo.MIN_POSITION,Utility_Constants.RELEASE_SERVO_DEFAULT);
        else if(gamepad1.dpad_up || gamepad2.dpad_up) releaseServoPos = Range.clip(releaseServoPos+Utility_Constants.SERVO_SENSITIVITY,releaseServo.MIN_POSITION,Utility_Constants.RELEASE_SERVO_DEFAULT);
        if(gamepad2.right_bumper) releaseServoPos = 0.64;
        else if(gamepad2.dpad_down) releaseServoPos = 0.38;
        if((runtime.milliseconds() >= 85000 && runtime.milliseconds() <= 90000) || (runtime.milliseconds() >= 115000 && runtime.milliseconds() <= 120000)) {
            blinkinPattern = RevBlinkinLedDriver.BlinkinPattern.BEATS_PER_MINUTE_LAVA_PALETTE;
            gamepad1.runRumbleEffect(endGameRumble);
        }
        else if(presetState != PRESET_STATE.NO_PRESET) blinkinPattern = RevBlinkinLedDriver.BlinkinPattern.BLUE_VIOLET;
        else if(distanceSensor.getDistance(DistanceUnit.MM) <= Utility_Constants.DISTANCE_SENSOR_THRESHOLD) blinkinPattern = RevBlinkinLedDriver.BlinkinPattern.BLUE_GREEN;
        else {
            blinkinPattern = RevBlinkinLedDriver.BlinkinPattern.RED_ORANGE;
        }
        if(distanceSensor.getDistance(DistanceUnit.MM) <= Utility_Constants.DISTANCE_SENSOR_THRESHOLD) {
            if(!isIntaked) {
                gamepad1.runRumbleEffect(boxSecured);
                intakeTimeout.reset();
            }
            forceSensorTimeout.reset();
            isIntaked = true;
            telemetry.addData("Force Sensor","Freight Detected");
            telemetry.update();
        } */
        /* else if(distanceSensor.getDistance(DistanceUnit.MM) <= Utility_Constants.DISTANCE_SENSOR_THRESHOLD) {
            if(!isIntaked) gamepad1.runRumbleEffect(boxSecured);
            isIntaked = true;
            blinkinPattern = RevBlinkinLedDriver.BlinkinPattern.GOLD;
            telemetry.addData("Distance Sensor","Freight Detected");
        } */
        /*else {
            isIntaked = false;
        }

        blinkin.setPattern(blinkinPattern); */

        //MOTOR SET POWER
        armDC.setTargetPosition((int) armPos);
        armDC.setPower(armPower);
        releaseServo.setPosition(releaseServoPos);
        mechanisms.rotateCarousel(carouselPower);
        mechanisms.maintainBalance();

        //TELEMETRY
        telemetry.addData("Status", "Looping");
        //telemetry.addData("Runtime", runtime.toString() + " Milliseconds"); //Display the runtime
        telemetry.addData("DCMotors", "armDC(%.2f)", armPos);
        telemetry.addData("Servos","releaseServoPos(%.2f)",releaseServoPos);
        telemetry.addData("Sensors","forceSensorVoltage(%.2f)",forceSensor.getVoltage());
        telemetry.update(); //Updates the telemetry

    }
}





