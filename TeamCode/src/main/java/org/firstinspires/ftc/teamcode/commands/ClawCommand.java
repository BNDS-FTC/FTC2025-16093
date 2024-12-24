package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.references.SSValues;
import org.firstinspires.ftc.teamcode.subsystems.ClawSubsystem;

public class ClawCommand extends InstantCommand {
    public ClawCommand(ClawSubsystem.clawState state) {
        super(
                () -> RobotHardware.getInstance().clawSubsystem.setClawPos(state)
        );
    }
}
