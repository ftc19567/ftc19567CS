package org.firstinspires.ftc.teamcode.mechanisms;


import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Intake implements MechanismTemplate{
    static DcMotorEx intakeMotor;

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



    }
    public void rollingIntake (double forward) {
        if (forward > 0.5) {
            intakeMotor.setPower(forward*2);
        }

    }

    public void ejection(boolean pressed) {
        if (pressed == true){
            intakeMotor.setPower(-0.5);
        } else {
            intakeMotor.setPower(0);
        }
    }
}
