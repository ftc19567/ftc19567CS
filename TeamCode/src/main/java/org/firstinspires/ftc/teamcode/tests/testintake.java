package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp
public class testintake extends OpMode {

    private DcMotor intakeMotor;

    @Override
    public void init() {
        intakeMotor = hardwareMap.get(DcMotor.class, "intakeMotor");
    }

    public void loop() {
        intakeMotor.setPower(gamepad1.left_stick_y);
    }
}
