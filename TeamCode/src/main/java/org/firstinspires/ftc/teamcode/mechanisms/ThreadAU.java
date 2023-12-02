package org.firstinspires.ftc.teamcode.mechanisms;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class ThreadAU extends Thread{

    static Arm arm;
    static Servo turnServo;

    @Override
    public void run() {
            turnServo.setPosition(1);
            try {
                sleep(100);
            } catch (InterruptedException e) {
            }
            arm.setPosition(0.6, 1686);
            try {
                sleep(700);
            } catch (InterruptedException e) {
            }
            turnServo.setPosition(0.2);

    }
}
