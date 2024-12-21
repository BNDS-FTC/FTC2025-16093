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

        pushTwoRedSamples();

        expPrepareForClawRedSampleUp(0,0);
        expClawRedSampleUp(0,0);

        expMoveToRedChamberPlace(4);
        highChamberPlace();

        expPrepareForClawRedSampleUp(17,-0.5);
        expClawRedSampleUp(17,-0.5);

        expMoveToRedChamberPlace(8);
        highChamberPlace();

        //simpleParkAtObservation();
        expPrepareForClawRedSampleUp(5,0.3);
        expClawRedSampleUp(5,0.3);

        expMoveToRedChamberPlace(10);
        highChamberPlace();
        prepareForTeleOpRed();

        while(opModeIsActive()){
            super.update.run();
        }

    }


}
