package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.uppersystems.SuperStructure;

@TeleOp(name = "TeleOp 16093")
public class TeleOp16093 extends LinearOpMode {
    public SuperStructure upper;
    @Override
    public void runOpMode(){
        upper = new SuperStructure(this);
        while(opModeIsActive()){

        }
    }
}
