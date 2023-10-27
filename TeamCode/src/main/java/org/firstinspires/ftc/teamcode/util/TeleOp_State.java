package org.firstinspires.ftc.teamcode.util;

public enum TeleOp_State {
    /**
     * Roadrunner is driving the robot somewhere
     */
    AUTOMATION_ROADRUNNER_MOVEMENT,
    /**
     * The robot is delivering the freight by rotating the flicker
     */
    AUTOMATION_FLICKER,
    /**
     * The driver has control of the robot (standard TeleOP)
     */
    DRIVER_CONTROL
}
