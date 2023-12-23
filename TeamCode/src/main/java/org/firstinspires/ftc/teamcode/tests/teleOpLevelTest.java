package org.firstinspires.ftc.teamcode.tests;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
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

    }
    public void loop() {

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

        if (gamepad2.a && !liftArmUp) {
            if (arm.highGetPosition() <= 1000) {
                turnServo.setPosition(0.85);
                timerUp.reset();
                timerUpB = true;
                liftArmUp = true;

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

        if (gamepad2.b && !liftArmDown) {
            if (arm.highGetPosition() > 1000) {
                turnServo.setPosition(1);
                arm.setPosition(1, 5);
                timerDownB = true;
                timerDown.reset();
                liftArmDown = true;
            }} else if (!gamepad2.b) {liftArmDown = false;}

        if (timerDown.milliseconds() >= 1000  && timerDownB == true) {
            turnServo.setPosition(0.67);
        }








    }

}
