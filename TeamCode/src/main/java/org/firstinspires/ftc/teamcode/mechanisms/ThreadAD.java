package org.firstinspires.ftc.teamcode.mechanisms;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class ThreadAD extends Thread {

    static Arm arm;
    static Servo turnServo;


    @Override
    public void run() {
            turnServo.setPosition(1);
            try {
                sleep(500);
            } catch (InterruptedException e) {
            }
            arm.setPosition(1, 5);
            try {
                sleep(1000);
            } catch (InterruptedException e) {
            }
            turnServo.setPosition(0.7);
    }
}
