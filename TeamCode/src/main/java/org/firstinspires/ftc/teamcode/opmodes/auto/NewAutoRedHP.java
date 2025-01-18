package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.AutoMaster;
//import org.firstinspires.ftc.teamcode.util.LastHeadingUtil;

@Autonomous
public class NewAutoRedHP extends AutoMaster{
    @Override
    public void runOpMode() throws InterruptedException {

        initHardware(new Pose2d(-15  ,62.3 ,Math.toRadians(90)));

        while(opModeInInit()){

        }

        waitForStart();
        newFirstMoveToRedChamberPlace();
        newResetCompletelyFromHighChamber();

        VexpPushThreeRedSamples();

        intakeRedSpecimenFromGround(0,0);

        newRedChamberPlace(6);
        newResetFromHighChamber();

        intakeRedSpecimenFromGround(0,0);

        newRedChamberPlace(9);
        newResetFromHighChamber();

        intakeRedSpecimenFromGround(0,0);

        newBlueChamberPlace(13);
        newResetFromHighChamber();

//        intakeSpecimenFromGround(0,0);
//
//        newBlueChamberPlace(3);
//        newResetFromHighChamber();

        newParkFromRedChamber();



//        intakeSpecimenFromGround(0,-3);
//
//        newBlueChamberPlace(9);
//        newResetFromHighChamber();
//
//
//        intakeSpecimenFromGround(0,-3);
//
//        newBlueChamberPlace(11);
//        newResetFromHighChamber();
//
//
//        intakeSpecimenFromGround(0,-3);
//
//        newBlueChamberPlace(13);
//        newResetFromHighChamber();


        while(opModeIsActive()){
            super.update.run();
        }

    }


}
