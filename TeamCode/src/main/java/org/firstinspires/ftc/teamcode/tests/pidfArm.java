package org.firstinspires.ftc.teamcode.tests;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@Config
@TeleOp
public class pidfArm extends OpMode {
    private PIDController controller;

    public static double p = 0.0047, i = 0.1, d = 0.0002; //PID as follwes
    public static double f = 0.03; //feedforward

    public static int target = 0; //to test target pos


    private final double ticks_per_degree = 537.7 * 0.20/ 180.0; //finding the amount of encoder ticks per degree in 180

    private DcMotorEx lowMotor, highMotor;

    @Override
    public void init() {
        controller = new PIDController(p, i, d);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        lowMotor = hardwareMap.get(DcMotorEx.class, "lowMotor");
        highMotor = hardwareMap.get(DcMotorEx.class, "highMotor");

        lowMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


    }

    public void loop() {

        controller.setPID(p, i, d);
        int lowArmPos = lowMotor.getCurrentPosition();
        double pid = controller.calculate(lowArmPos, target);
        double ff = Math.cos(Math.toRadians(target/ticks_per_degree)) * f;

        double power = pid + ff;

        lowMotor.setPower(power);
        highMotor.setPower(power);

        telemetry.addData("pos : ", lowArmPos);
        telemetry.addData("target : ", target);
        telemetry.update();

    }
}
