package org.firstinspires.ftc.teamcode.mechanisms;

//import static org.firstinspires.ftc.teamcode.util.UtilConstants.clawIntakePos;
//import static org.firstinspires.ftc.teamcode.util.UtilConstants.clawOuttakePos;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Claw implements MechanismTemplate{
    Servo clawServo;
    public Claw(HardwareMap hardwareMap, Telemetry telemetry)
    {
        setMode(hardwareMap, telemetry);
    }
    @Override
    public void setMode(HardwareMap hwMap, Telemetry telemetry) {
        clawServo = hwMap.get(Servo.class, "clawServo");
        clawServo.setDirection(Servo.Direction.FORWARD);
    }

    public void position(double pos){
        clawServo.setPosition(Range.clip(pos,0,1));
    }


    public double getPos(){
        return clawServo.getPosition();
    }

    public void open(){
        //clawServo.setPosition(clawOuttakePos);
    }

    public void close(){
        //clawServo.setPosition(clawIntakePos);
    }

    public int portNum(){
        return clawServo.getPortNumber();
    }
}