//package org.firstinspires.ftc.teamcode.opmodes.auto;
//
//import com.acmerobotics.roadrunner.geometry.Pose2d;
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//
//import org.firstinspires.ftc.teamcode.AutoMaster;
//
//@Autonomous
//public class AutoRedHP extends AutoMaster{
//    @Override
//    public void runOpMode() throws InterruptedException {
//
//        initHardware(new Pose2d(15  ,-62.3 ,Math.toRadians(-90)));
//
//        while(opModeInInit()){
//
//        }
//
//        waitForStart();
//        firstMoveToRedChamberPlace(1);
//        highChamberPlace();
//
//        //        prepareForPushTwoBlueSample();
//        pushTwoRedSamples();
//
//        prepareForClawRedSampleUp(0,0);
//        clawRedSampleUp(0,0);
//
//        moveToRedChamberPlace(4);
//        highChamberPlace();
//
//        prepareForClawRedSampleUp(17,-0.5);
//        clawRedSampleUp(17,-0.5);
//
//        moveToRedChamberPlace(8);
//        highChamberPlace();
//
//        //simpleParkAtObservation();
//        prepareForClawRedSampleUp(5,0.3);
//        clawRedSampleUp(5,0.3);
//        prepareForTeleOpRed();
//
//        while(opModeIsActive()){
//            super.update.run();
//        }
//
//    }
//
//
//}
