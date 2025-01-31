package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.AutoMaster;

@Autonomous
public class NewAutoBlueHP extends AutoMaster{
    //TODO: Make a version w/o color sensor
    @Override
    public void runOpMode() throws InterruptedException {

        initHardware(new Pose2d(-15  ,62.3 ,Math.toRadians(90)));

        while(opModeInInit()){

        }

        waitForStart();
        newFirstMoveToBlueChamberPlace();
        newResetCompletelyFromHighChamber();

        VexpPushTwoBlueSamples();
//        intakeThreeBlueSamples();

        firstIntakeSpecimenFromGround(-2,-3);
        newBlueChamberPlace(13,0);
        newResetFromHighChamber();

        intakeSpecimenFromGround(-2,-3);
        newBlueChamberPlace(10,0);
        newResetFromHighChamber();

        intakeSpecimenFromGround(-2,-3);
        newBlueChamberPlace(7,0);
        newResetFromHighChamber();

        intakeSpecimenFromGround(-2,-3);
        newBlueChamberPlace(5,-1.3);
        newResetFromHighChamber();

//        intakeSpecimenFromGround(-1,-2);

        newParkFromBlueChamber();

        while(opModeIsActive()){
            super.update.run();
        }

    }


}