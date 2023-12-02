package org.firstinspires.ftc.teamcode.teleop;



import static java.lang.Thread.sleep;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.mechanisms.Arm;
import org.firstinspires.ftc.teamcode.mechanisms.Intake;
//import org.firstinspires.ftc.teamcode.mechanisms.ThreadAD;
//import org.firstinspires.ftc.teamcode.mechanisms.ThreadAU;


@TeleOp
public class TeleOPMeet1 extends OpMode {


    private DcMotor frontLeftMotor;
    private DcMotor backLeftMotor;
    private DcMotor frontRightMotor;
    private DcMotor backRightMotor;
    public Servo turnServo;
    private Servo planeServo;
    public Arm arm;
    private DcMotor lowMotor, highMotor;
    private Intake intake;
    private boolean liftArmUp = false;
    private boolean liftArmDown = false;
    private boolean boxUp = false;
    private boolean boxDown = false;
    private boolean reset = false;
    private boolean plane = false;
    private boolean hanging = false;
    private boolean hang = false;
    private boolean release = false;

    private boolean autoTurnServo = true;

    private double servoPosCalc = 1;


    //ThreadAU threadAU = new ThreadAU();
    //ThreadAD threadAD = new ThreadAD();




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


        lowMotor = hardwareMap.get(DcMotor.class, "lowMotor");
        highMotor = hardwareMap.get(DcMotor.class, "highMotor");



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

        //TURNING SERVO
        turnServo = hardwareMap.get(Servo.class, "turnServo");

        turnServo.setPosition(0.7);
        Arm.setPosition(1, 0);

        telemetry.addData("Encoder :", arm.lowGetPosition());

        telemetry.update();
        //AIRPLANE
        planeServo = hardwareMap.get(Servo.class, "planeServo");
        planeServo.setPosition(planeServo.getPosition());



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
        if (gamepad2.a && !liftArmUp) {
            if (arm.highGetPosition() <= 1000) {
                turnServo.setPosition(1);
                try {
                    Thread.sleep(100);
                } catch (InterruptedException e) {
                    throw new RuntimeException(e);
                }
                arm.setPosition(0.6, 1856);
                try {
                    Thread.sleep(700);
                } catch (InterruptedException e) {
                    throw new RuntimeException(e);
                }
                turnServo.setPosition(0.11000000000000004);
                liftArmUp = true;
            }
        }else if (!gamepad2.a) {
            liftArmUp = false;
        }

        //LiftArmDown

        if (gamepad2.b & !liftArmDown) {
            if (arm.highGetPosition() >= 1000) {
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
                turnServo.setPosition(0.7);


                liftArmDown = true;

            }
        } else if (!gamepad2.b) {
            liftArmDown = false;
        }

        //RESET ARM ENCODER
        if (gamepad1.y && !reset) {
            lowMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            highMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            reset = true;
        } else if (!gamepad1.y) {
            reset = false;
        }
/*
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
                turnServo.setPosition(0.7);

        */


        if (gamepad2.left_bumper) {
            Arm.negativeArmPower();

        }


        if (gamepad2.left_trigger > 0.5) {
            Arm.positiveArmPower();

        }

        //Turning Servo
        if (gamepad2.dpad_down && !boxDown) {
            turnServo.setPosition(turnServo.getPosition() + 0.03);
            boxDown = true;
        } else if (!gamepad2.dpad_down) {
            boxDown = false;
        }
        if (gamepad2.dpad_up && !boxUp) {
            turnServo.setPosition(turnServo.getPosition() - 0.03);
            boxUp = true;
        } else if (!gamepad2.dpad_up) {
            boxUp = false;
        }



/*
        //turning servo
        if (arm.lowGetPosition() > 1000) {
            servoPosCalc = 0.7 - (arm.lowGetPosition()/1600 * 0.35);
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
        */




        //intake
        intake = new Intake(hardwareMap, telemetry);

        if (gamepad2.right_trigger > 0) {
            intake.rollingIntake(1);
        }



        //ejects pixels
        intake.ejection((gamepad2.right_bumper));


        //airplane
        if (gamepad1.a && !plane) {
            planeServo.setPosition(planeServo.getPosition()+0.5);
            //TUNE VALUE ABOVE TO GET RIGHT VALUE FOR RELEASED SERVO POSITION22
            plane = true;
        } else if (!gamepad1.a) {
            plane = false;
        }


        //hanging
        if (gamepad1.y && !hanging) {
            int i = 1;
            if(i == 1) {
                turnServo.setPosition(1);
                try {
                    sleep(300);
                } catch (InterruptedException e) {
                    throw new RuntimeException(e);
                }
                Arm.setPosition(1, 1100);
                //TUNE ABOVE VALUE FOR WHEN ARM IS FULLY STRAIGHT
                try {
                    sleep(500);
                } catch (InterruptedException e) {
                    throw new RuntimeException(e);
                }
                turnServo.setPosition(0.3);
                //TUNE VALUE FOR STRAIGHT BOX
            }
            hanging = true;
        } else if (!gamepad1.y) {
            hanging = false;
        }

        //release turning servo
        if (gamepad1.b && !release) {
            turnServo.setPosition(0.15);
            try {
                sleep(150);
            } catch (InterruptedException e) {
                throw new RuntimeException(e);
            }
            turnServo.getController().pwmDisable();
            release = true;
        } else if (!gamepad1.b) {
            release = false;
        }
        //hang
        if (gamepad1.x && !hang) {
            Arm.setPosition(1, 100);
            hang = true;
        } else if (!gamepad1.x) {
            hang = false;
        }




        telemetry.addData("ServoPos : ", turnServo.getPosition());
        telemetry.addData("ArmPos : ", lowMotor.getCurrentPosition());
        telemetry.addData("liftUpB : ", liftArmUp);
        telemetry.addData("liftDownB : ", liftArmDown);

    }
}