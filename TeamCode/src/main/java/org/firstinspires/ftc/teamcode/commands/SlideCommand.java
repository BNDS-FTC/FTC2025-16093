package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.subsystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.SlideSubsystem;

public class SlideCommand extends CommandBase {

    // The subsystem the command runs on
    private final SlideSubsystem mSlideSubsystem;
    int targetPosition;

    public SlideCommand(int pos) {
        mSlideSubsystem = RobotHardware.getInstance().slideSubsystem;
        targetPosition = pos;
        addRequirements(mSlideSubsystem);
    }

    @Override
    public void initialize() {
        mSlideSubsystem.setSlidesByP(targetPosition,1);
    }

    @Override
    public boolean isFinished() {
        if((Math.abs(mSlideSubsystem.getSlideError()) < 30)){
            return true;
        }else{
            return false;
        }
    }

}