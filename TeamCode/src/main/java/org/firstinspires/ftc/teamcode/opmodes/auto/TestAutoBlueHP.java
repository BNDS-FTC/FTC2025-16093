//package org.firstinspires.ftc.teamcode.opmodes.auto;
//
//import com.acmerobotics.roadrunner.geometry.Pose2d;
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//
//import org.firstinspires.ftc.teamcode.opmodes.auto.AutoMaster;
//
//@Autonomous
//public class TestAutoBlueHP extends AutoMaster{
//    //TODO: Make a version w/o color sensor
//    @Override
//    public void runOpMode() throws InterruptedException {
//
//        initAuto(new Pose2d(-15  ,62.3 ,Math.toRadians(90)));
//
//        while(opModeInInit()){
//
//        }
//
//        waitForStart();
//        newFirstMoveToBlueChamberPlace();
//
//        intakeSpecimenFromBlueWall(0,0);
//        blueChamberPlaceFromWall(13,0);
//
//        intakeSpecimenFromBlueWall(0,-0);
//        blueChamberPlaceFromWall(12,-1);
//
//        intakeSpecimenFromBlueWall(-1,0);
//        blueChamberPlaceFromWall(10,-1);
//
//        intakeSpecimenFromBlueWall(0,-1);
//        blueChamberPlaceFromWall(8,-1);
//
//        intakeSpecimenFromBlueWall(0,-1);
//        blueChamberPlaceFromWall(5,-1);
//
////        intakeSpecimenFromWall(-2);
//
////        intakeSpecimenFromGround(-1,-0);
////        newBlueChamberPlace(12,-0);
////        newResetFromHighChamber();
////
////        intakeSpecimenFromGround(-1,-0);
////        newBlueChamberPlace(11,-0);
////        newResetFromHighChamber();
////
////        intakeSpecimenFromGround(-1,-1);
////        newBlueChamberPlace(8,-0);
////        newResetFromHighChamber();
////
////        intakeSpecimenFromGround(-1,-1);
////        newBlueChamberPlace(5,-0);
////        newResetFromHighChamber();
////
////        intakeSpecimenFromGround(-1,-1);
////        newBlueChamberPlace(2,-0);
////        newResetFromHighChamber();
//
//        while(opModeIsActive()){
//            super.update.run();
//        }
//
//    }
//
//
//}