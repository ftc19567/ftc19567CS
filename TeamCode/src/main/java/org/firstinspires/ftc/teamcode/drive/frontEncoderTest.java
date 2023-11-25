package org.firstinspires.ftc.teamcode.drive;


import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.localization.ThreeTrackingWheelLocalizer;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.util.Encoder;

import java.util.Arrays;
import java.util.List;

import org.firstinspires.ftc.teamcode.util.Encoder;

@TeleOp
public class frontEncoderTest extends OpMode{
    private Encoder frontEncoder, leftEncoder, rightEncoder;

    public static double TICKS_PER_REV = 2000;
    public static double WHEEL_RADIUS = 0.945; // in
    public static double GEAR_RATIO = 1; // output (wheel) speed / input (encoder) speed

    public static double LATERAL_DISTANCE =  8.846331283631548; // in; distance between the left and right wheels
    public static double FORWARD_OFFSET = -4; // in; offset of the lateral wheel

    public static double X_MULTIPLIER = 1.00659115964; // Multiplier in the X direction
    public static double Y_MULTIPLIER = 1;

    private List<Integer> lastEncPositions, lastEncVels;

    public static double encoderTicksToInches(double ticks) {
        return WHEEL_RADIUS * 2 * Math.PI * GEAR_RATIO * ticks / TICKS_PER_REV;
    }

    public void init() {
        leftEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "intakeMotor"));
        rightEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "backRightMotor"));
        frontEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "frontLeftMotor"));
        frontEncoder.setDirection(Encoder.Direction.REVERSE);
        rightEncoder.setDirection(Encoder.Direction.REVERSE);
    }

    public void loop() {
        int leftPos = leftEncoder.getCurrentPosition();
        int rightPos = rightEncoder.getCurrentPosition();
        int frontPos = frontEncoder.getCurrentPosition();
        telemetry.addData("raw front: ", frontPos);
        telemetry.addData("raw left : ", leftPos);
        telemetry.addData("raw right : ", rightPos);
        telemetry.update();

    }
}
