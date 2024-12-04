package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.references.Globals;
import org.firstinspires.ftc.teamcode.references.SSValues;

public class ResetFromScoringCommand extends SequentialCommandGroup {
    public ResetFromScoringCommand() {
        super(
                new InstantCommand(Globals::stopScoring),
                new WristCommand(SSValues.WRIST_INTAKE),
                new WaitCommand(250),
                new SlideCommand(SSValues.SLIDE_MIN),
                new ArmCommand(SSValues.ARM_DOWN),
                new WristCommand(SSValues.WRIST_DEFAULT)
        );
    }
}

