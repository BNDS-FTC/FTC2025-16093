package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.RunCommand;

import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;

public class IntakeCommand extends RunCommand {
    public IntakeCommand(double pos){
        super(
                () -> RobotHardware.getInstance().intakeSubsystem.setIntake(pos)
        );
    }
}
