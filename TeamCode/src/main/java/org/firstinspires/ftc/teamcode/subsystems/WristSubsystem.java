package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class WristSubsystem extends SubsystemBase {

    private final Servo wrist;


    public WristSubsystem(HardwareMap hardwareMap) {
        wrist = hardwareMap.get(Servo.class,"wrist");

    }

    public void setWristPos(double pos){
        wrist.setPosition(pos);
    }
}