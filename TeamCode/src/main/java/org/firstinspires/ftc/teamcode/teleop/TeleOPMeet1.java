package org.firstinspires.ftc.teamcode.teleop;



import static java.lang.Thread.sleep;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

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
    public Servo planeServo;
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

    private boolean timerUpB = false;

    private boolean timerUpFB = false;

    private boolean timerDownB = false;

    private double powerMult = 1;





    ElapsedTime timerUp = new ElapsedTime();
    ElapsedTime timerUpFinsh = new ElapsedTime();
    ElapsedTime timerDown = new ElapsedTime();


    //ThreadAU threadAU = new ThreadAU();
    //ThreadAD threadAD = new ThreadAD();




    @Override
    public void init() {

        //MECANUM
        frontLeftMotor = hardwareMap.get(DcMotor.class, "frontLeftMotor");
        backLeftMotor = hardwareMap.get(DcMotor.class, "backLeftMotor");
        frontRightMotor = hardwareMap.get(DcMotor.class, "frontRightMotor");
        backRightMotor = hardwareMap.get(DcMotor.class, "backRightMotor");

        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Reverse the right side motors. This may be wrong for your setup.
        // If your robot moves backwards when commanded to go forwards,
        // reverse the left side instead.
        // See the note about this earlier on this page.
        frontLeftMotor.setDirection(DcMotor.Direction.REVERSE);


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

        turnServo.setPosition(0.67);
        Arm.setPosition(1, 0);

        telemetry.addData("Encoder :", arm.lowGetPosition());

        //AIRPLANE
        planeServo = hardwareMap.get(Servo.class, "planeServo");
        planeServo.setPosition(1);

        telemetry.update();



    }
    // Declare our motors
    // Make sure your ID's match your configuration



    @Override
    public void loop() {

        if (gamepad1.left_bumper) {
            powerMult = 0.5;
            telemetry.addData("Current Speed : ", (powerMult * 100) + "%");
        }

        if (gamepad1.right_bumper) {
            powerMult = 1;
            telemetry.addData("Current Speed : ", (powerMult * 100) + "%");
        }



        //Mecanum
        double y = -gamepad1.left_stick_y * powerMult ; // Remember, Y stick value is reversed
        double x  = gamepad1.left_stick_x * 1.1 * powerMult; // Counteract imperfect strafing
        double rx = gamepad1.right_stick_x * 0.5 * powerMult;

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


        if (gamepad2.a && !liftArmUp && timerUpB == false && timerUpFB == false) {
            if (arm.highGetPosition() <= 1000) {
                turnServo.setPosition(1);
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
            turnServo.setPosition(0.24);
            timerUpFB = false;
        }

        /*
        //ARM
        if (gamepad2.a && !liftArmUp) {
            if (arm.highGetPosition() <= 1000) {
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
                liftArmUp = true;
            }
        }else if (!gamepad2.a) {
            liftArmUp = false;
        }

        
         */

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

        /*
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
                turnServo.setPosition(0.67);


                liftArmDown = true;

            }
        } else if (!gamepad2.b) {
            liftArmDown = false;
        }

         */

        //RESET ARM ENCODER
        /*
        if (gamepad1.y && !reset) {
            lowMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            highMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            reset = true;
        } else if (!gamepad1.y) {
            reset = false;
        }
        */

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
            planeServo.setPosition(0.5);
            plane = true;
        } else if (!gamepad1.a){
            plane = false;
        }


        //hanging
        if (gamepad1.x && !hanging) {
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
        } else if (!gamepad1.x) {
            hanging = false;
        }

        //release turning servo
        if (gamepad1.y && !release) {
            turnServo.setPosition(0.15);
            try {
                sleep(150);
            } catch (InterruptedException e) {
                throw new RuntimeException(e);
            }
            turnServo.getController().pwmDisable();
            release = true;
        } else if (!gamepad1.y) {
            release = false;
        }
        //hang
        if (gamepad1.b && !hang) {
            Arm.setPosition(1, 100);
            hang = true;
        } else if (!gamepad1.b) {
            hang = false;
        }




        telemetry.addData("ServoPos : ", turnServo.getPosition());
        telemetry.addData("ArmPos : ", lowMotor.getCurrentPosition());
        telemetry.addData("liftUpB : ", liftArmUp);
        telemetry.addData("liftDownB : ", liftArmDown);
        telemetry.addData("planeServo Pos : ", planeServo.getPosition());

    }
}