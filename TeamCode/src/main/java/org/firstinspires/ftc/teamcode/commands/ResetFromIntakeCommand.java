package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import org.firstinspires.ftc.teamcode.references.Globals;
import org.firstinspires.ftc.teamcode.references.SSValues;

public class ResetFromIntakeCommand extends SequentialCommandGroup {
    public ResetFromIntakeCommand() {
        super(
                new InstantCommand(Globals::stopIntaking),
                new GrabCommand(SSValues.GRAB_CLOSED),
                new WristCommand(SSValues.WRIST_DEFAULT),
                new WaitCommand(250),
                new SlideCommand(SSValues.SLIDE_MIN)
        );
    }
}

