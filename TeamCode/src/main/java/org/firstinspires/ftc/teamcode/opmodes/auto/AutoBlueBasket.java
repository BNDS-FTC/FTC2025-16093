//package org.firstinspires.ftc.teamcode.opmodes.auto;
//
//import com.acmerobotics.roadrunner.geometry.Pose2d;
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//
//import org.firstinspires.ftc.teamcode.AutoMaster;
//
//@Autonomous
//public class AutoBlueBasket extends AutoMaster{
//    @Override
//    public void runOpMode() throws InterruptedException {
//
//        initHardware(new Pose2d(30, 62.3, Math.toRadians(-90)));
//
//        while(opModeInInit()){
//
//        }
//
//        waitForStart();
//        firstPutBlueBasket();
//        resetAfterBlueBasketAndMoveToIntake(-0.5,0);
//
//        getYellowSamples();
//
//        putBlueBasketFromGround(0,0, 0);
//        resetAfterBlueBasketAndMoveToIntake(10,-2);
//
//        getYellowSamples();
//
//        putBlueBasketFromGround(0,0, 0);
//        reset();
//
//        moveAndIntakeLastBasketSampleBlue();
////        reset();
//
//        putBlueBasketFromGround(0,0, 0);
//
//        hangFromBlueBasket();
//
////        parkAtBlueObservationFromBasket();
////        prepareForTeleOp();
//
//        while(opModeIsActive()){
//            super.update.run();
//        }
//
//
////        reset();
//    }
//
//
//}
