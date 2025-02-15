package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.AutoMaster;

@Autonomous
public class TestAutoBlueHP extends AutoMaster{
    //TODO: Make a version w/o color sensor
    @Override
    public void runOpMode() throws InterruptedException {

        initHardware(new Pose2d(-15  ,62.3 ,Math.toRadians(90)));

        while(opModeInInit()){

        }

        waitForStart();
        newFirstMoveToBlueChamberPlace();

        intakeSpecimenFromWall(0);
        blueChamberPlaceFromWall(10,0);

        intakeSpecimenFromWall(-0);
        blueChamberPlaceFromWall(8,-3);

        intakeSpecimenFromWall(-0);
        blueChamberPlaceFromWall(6,-3);

        intakeSpecimenFromWall(1);
        blueChamberPlaceFromWall(4,-3);

        intakeSpecimenFromWall(1);
        blueChamberPlaceFromWall(4,-3);

        intakeSpecimenFromWall(1);

//        intakeSpecimenFromGround(-1,-0);
//        newBlueChamberPlace(12,-0);
//        newResetFromHighChamber();
//
//        intakeSpecimenFromGround(-1,-0);
//        newBlueChamberPlace(11,-0);
//        newResetFromHighChamber();
//
//        intakeSpecimenFromGround(-1,-1);
//        newBlueChamberPlace(8,-0);
//        newResetFromHighChamber();
//
//        intakeSpecimenFromGround(-1,-1);
//        newBlueChamberPlace(5,-0);
//        newResetFromHighChamber();
//
//        intakeSpecimenFromGround(-1,-1);
//        newBlueChamberPlace(2,-0);
//        newResetFromHighChamber();

        while(opModeIsActive()){
            super.update.run();
        }

    }


}