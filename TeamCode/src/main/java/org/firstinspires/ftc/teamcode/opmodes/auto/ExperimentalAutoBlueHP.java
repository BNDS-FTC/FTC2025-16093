package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.AutoMaster;

@Autonomous
public class ExperimentalAutoBlueHP extends AutoMaster{
    @Override
    public void runOpMode() throws InterruptedException {

        initHardware(new Pose2d(15  ,-62.3 ,Math.toRadians(-90)));

        while(opModeInInit()){

        }

        waitForStart();
        expFirstMoveToBlueChamberPlace(1);
        highChamberPlace();

        expPushTwoBlueSamples();

        expPrepareForClawBlueSampleUp(0,0);
        expClawBlueSampleUp(0,0);

        expMoveToBlueChamberPlace(3);
        highChamberPlace();

        expPrepareForClawBlueSampleUp(17,-0.5);
        expClawRedSampleUp(17,-0.5);

        expMoveToBlueChamberPlace(6);
        highChamberPlace();

        expPrepareForClawBlueSampleUp(5,0.3);
        expClawRedSampleUp(5,0.3);

        expMoveToBlueChamberPlace(9);
        highChamberPlace();
        expPrepareForTeleOpBlue();

        while(opModeIsActive()){
            super.update.run();
        }

    }


}
