package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.AutoMaster;

@Autonomous
public class ExperimentalAutoRedHP extends AutoMaster{
    @Override
    public void runOpMode() throws InterruptedException {

        initHardware(new Pose2d(15  ,-62.3 ,Math.toRadians(-90)));

        while(opModeInInit()){

        }

        waitForStart();
        expFirstMoveToRedChamberPlace(1);
        highChamberPlace();

        VexpPushTwoRedSamples();

        expPrepareForClawRedSampleUp(0,0,0);
        expClawRedSampleUp(0,0);

        expMoveToRedChamberPlace(3);
        highChamberPlace();

        expPrepareForClawRedSampleUp(17,-0.5,200);
        expClawRedSampleUp(17,-0.5);

        expMoveToRedChamberPlace(6);
        highChamberPlace();

        expPrepareForClawRedSampleUp(17,-0.1,0);
        expClawRedSampleUp(17,-0.1);

        expMoveToRedChamberPlace(9);
        highChamberPlace();
        expPrepareForTeleOpRed();

        while(opModeIsActive()){
            super.update.run();
        }

    }


}
