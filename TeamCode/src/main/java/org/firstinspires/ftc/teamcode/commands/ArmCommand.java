package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.subsystems.ArmSubsystem;

public class ArmCommand extends CommandBase {

    // The subsystem the command runs on
    private final ArmSubsystem mArmSubsystem;
    int targetPosition;

    public ArmCommand(int pos) {
        mArmSubsystem = RobotHardware.getInstance().armSubsystem;
        targetPosition = pos;
        addRequirements(mArmSubsystem);
    }

    @Override
    public void initialize() {
        mArmSubsystem.setArmByP(targetPosition,1);
    }

    @Override
    public boolean isFinished() {
        if((Math.abs(mArmSubsystem.getArmError()) < 20)){
            return true;
        }else{
            return false;
        }
    }

}