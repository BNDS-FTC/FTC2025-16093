package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.AutoMaster;

@Autonomous
public class NewerAutoBlueHp extends AutoMaster{
    //These class names are getting kind of ridiculous.
    @Override
    public void runOpMode() throws InterruptedException {

        initHardware(new Pose2d(-15  ,62.3 ,Math.toRadians(90)));

        while(opModeInInit()){

        }

        waitForStart();
        newFirstMoveToBlueChamberPlace();
        newResetCompletelyFromHighChamber();

//        expPushTwoBlueSamples();

        intakeSpecimenFromWall(-3,-3);

        newerBlueChamberPlace(3);
//        newResetFromHighChamber();
//
//        intakeSpecimenFromWall(-2,-1);
//
//        newBlueChamberPlace(6);
//        newResetFromHighChamber();
//
//        intakeSpecimenFromWall(-2,-3);
//
//        newBlueChamberPlace(8);
//        newResetFromHighChamber();
//
//        intakeSpecimenFromWall(-2,-3);
//
//        newBlueChamberPlace(10);
//        newResetFromHighChamber();
//
//        newParkFromBlueChamber();




        while(opModeIsActive()){
            super.update.run();
        }

    }


}
