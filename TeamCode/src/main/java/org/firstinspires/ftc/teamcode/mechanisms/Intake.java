package org.firstinspires.ftc.teamcode.mechanisms;


import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Intake implements MechanismTemplate{
    static DcMotorEx intakeMotor;
    static Servo dumperServo;

    Telemetry telemetry;

    public Intake(HardwareMap hwMap, Telemetry telemetry) {
            setMode(hwMap, telemetry);
        }

    @Override
    public void setMode(HardwareMap hwMap, Telemetry telemetry) {
        this.telemetry = telemetry;
        intakeMotor = hwMap.get(DcMotorEx.class, "lowMotor");
        intakeMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeMotor = hwMap.get(DcMotorEx.class, "intakeMotor");
        dumperServo = hwMap.get(Servo.class, "dumperServo");


    }
    public void rollingIntake (String open, String on) {


        if (on == "true") {
            intakeMotor.setPower(1);
        } else if(on == "false") {
            intakeMotor.setPower(0);
        } else if (on == "reverse") {
            intakeMotor.setPower(-1);
        }

        if (open == "true") {
            dumperServo.setPosition(1);
        } else if (open == "false") {
            dumperServo.setPosition(0);
        } else {
            //does nothing
        }

    }
}
