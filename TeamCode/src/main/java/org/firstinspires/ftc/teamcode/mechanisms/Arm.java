package org.firstinspires.ftc.teamcode.mechanisms;


import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Arm implements MechanismTemplate{
    static DcMotorEx lowMotor;
    static DcMotorEx highMotor;
    Telemetry telemetry;

    //private VoltageSensor voltageSensor;
    //private double voltageComp = 1.0;

    public Arm (HardwareMap hwMap, Telemetry telemetry){
        setMode(hwMap, telemetry);
    }

    @Override
    public void setMode(HardwareMap hwMap, Telemetry telemetry) {
        this.telemetry = telemetry;
        lowMotor = hwMap.get(DcMotorEx.class, "lowMotor");
        lowMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        lowMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lowMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lowMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        highMotor = hwMap.get(DcMotorEx.class, "lowMotor");
        highMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        highMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        highMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        highMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //voltageSensor = hwMap.voltageSensor.iterator().next();
        //voltageComp = 12.0 / voltageSensor.getVoltage();

    }

    public static void setPosition(double pow, int pos){
        lowMotor.setPower(pow);
        lowMotor.setTargetPosition(Range.clip(pos,0,2000));
        lowMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        highMotor.setPower(pow);
        highMotor.setTargetPosition(Range.clip(pos,0,2000));
        highMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void setPower(double pow) {
        lowMotor.setPower(pow);
        highMotor.setPower(pow);
    }

    /*public double getVoltageComp(){
        return voltageComp;
    }*/

    /*public int getPosition(){
        return lowMotor.getTargetPosition();
        return highMotor.getTargetPosition();
    } */

    //public double getVelocity() {return verticalMotor.getVelocity(); }
    public int lowGetPosition() {
        return lowMotor.getTargetPosition();
    }

    public int highGetPosition() {
        return highMotor.getTargetPosition();
    }


    public void resetPosition() {
        lowMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lowMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lowMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        highMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        highMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        highMotor.setDirection(DcMotorSimple.Direction.FORWARD);

    }

    public void stop(){
        lowMotor.setPower(0);
        highMotor.setPower(0);
    }

    public static void negativeArmPower() {
        lowMotor.setTargetPosition(lowMotor.getCurrentPosition() - 50);
        highMotor.setTargetPosition(highMotor.getCurrentPosition() - 50);
        lowMotor.setPower(-0.8);
        highMotor.setPower(-0.8);
        lowMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        highMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public static void positiveArmPower() {

        lowMotor.setTargetPosition(lowMotor.getCurrentPosition() + 50);
        highMotor.setTargetPosition(highMotor.getCurrentPosition() + 50);
        lowMotor.setPower(0.8);
        highMotor.setPower(0.8);
        lowMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        highMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }




}