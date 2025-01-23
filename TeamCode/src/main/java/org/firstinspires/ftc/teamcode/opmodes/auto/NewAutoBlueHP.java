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

        firstIntakeSpecimenFromGround(0,0);
        newBlueChamberPlace(12,0);
        newResetFromHighChamber();

        intakeSpecimenFromGround(-2,-3);

        newBlueChamberPlace(8,1);
        newResetFromHighChamber();

        intakeSpecimenFromGround(-3,-4);

        newBlueChamberPlace(5,0);
        newResetFromHighChamber();

        intakeSpecimenFromGround(-3,-5);

        newBlueChamberPlace(3,0);
        newResetFromHighChamber();

//        newParkFromBlueChamber();

        while(opModeIsActive()){
            super.update.run();
        }

    }


}