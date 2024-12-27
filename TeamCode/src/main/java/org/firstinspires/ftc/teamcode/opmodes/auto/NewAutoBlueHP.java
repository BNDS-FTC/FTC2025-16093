package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.AutoMaster;

@Autonomous
public class NewAutoBlueHP extends AutoMaster{
    @Override
    public void runOpMode() throws InterruptedException {

        initHardware(new Pose2d(-15  ,62.3 ,Math.toRadians(90)));

        while(opModeInInit()){

        }

        waitForStart();
        newFirstMoveToBlueChamberPlace();
        newResetFromHighChamber();

        intakeSpecimenFromGround(-2,-3);

        newBlueChamberPlace(3);
        newResetFromHighChamber();

        intakeSpecimenFromGround(-2,-3);

        newBlueChamberPlace(6);
        newResetFromHighChamber();

        intakeSpecimenFromGround(0,-3);

        newBlueChamberPlace(8);
        newResetFromHighChamber();

        intakeSpecimenFromGround(0,-3);

        newBlueChamberPlace(10);
        newResetFromHighChamber();

        newParkFromBlueChamber();


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
