package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class GrabSubsystem extends SubsystemBase {

    private final Servo mGrab;


    public GrabSubsystem(HardwareMap hardwareMap) {
        mGrab = hardwareMap.get(Servo.class,"grab");
        mGrab.setDirection(Servo.Direction.REVERSE);
    }

    public void setGrabPos(double pos){
        mGrab.setPosition(pos);
    }
}