package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.subsystems.ClawSubsystem;

public class GrabCommand extends InstantCommand{
    public GrabCommand(double pos) {
        super(
                () -> RobotHardware.getInstance().grabSubsystem.setGrabPos(pos)
        );
    }
}
