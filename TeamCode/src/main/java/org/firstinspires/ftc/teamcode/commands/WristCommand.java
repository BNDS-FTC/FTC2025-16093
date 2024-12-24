package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.RobotHardware;

public class WristCommand extends InstantCommand{
    public WristCommand(double pos) {
        super(
                () -> RobotHardware.getInstance().wristSubsystem.setWristPos(pos)
        );
    }
}
