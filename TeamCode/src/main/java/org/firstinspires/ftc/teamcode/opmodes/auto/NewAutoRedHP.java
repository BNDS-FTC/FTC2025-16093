package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.AutoMaster;

@Autonomous
public class NewAutoRedHP extends AutoMaster{
    @Override
    public void runOpMode() throws InterruptedException {

        initHardware(new Pose2d(15,-62.3 ,Math.toRadians(-90)));

        while(opModeInInit()){

        }

        waitForStart();
        newFirstMoveToRedChamberPlace();
        newResetCompletelyFromHighChamber();

        VexpPushTwoRedSamples();

        intakeSpecimenFromGround(0,0);

        newRedChamberPlace(10);
        newResetFromHighChamber();

        intakeSpecimenFromGround(0,0);

        newRedChamberPlace(7);
        newResetFromHighChamber();

        intakeSpecimenFromGround(0,0);

        newRedChamberPlace(5);
        newResetFromHighChamber();

        intakeSpecimenFromGround(0,0);

        newRedChamberPlace(3);
        newResetFromHighChamber();

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