package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public interface MechanismTemplate {
    public void setMode(HardwareMap hwMap, Telemetry telemetry);
}