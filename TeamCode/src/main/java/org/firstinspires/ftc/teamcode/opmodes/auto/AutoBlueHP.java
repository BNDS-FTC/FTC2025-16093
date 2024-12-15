package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.AutoMaster;

@Autonomous
public class AutoBlueHP extends AutoMaster{
    @Override
    public void runOpMode() throws InterruptedException {
        side_color = BLUE;
        startSide = NEGATIVE;

        initHardware(new Pose2d(-15  ,62.3 ,Math.toRadians(90)));

        while(opModeInInit()){

        }

        waitForStart();
        moveToBlueChamberPlace(0);
        highChamberPlace();

        prepareForPushTwoBlueSample();
        pushTwoBlueSamples();
//        reset();
        prepareForClwBlueSampleUp();
        clawBlueSampleUp();
        moveToBlueChamberPlace(3);
        highChamberPlace();

        //prepareForTeleOp();

        //reset();
        prepareForClwBlueSampleUp();
        clawBlueSampleUp();
        //这下面的代码临时的

//        moveToBlueChamberPlace(6);
//        highChamberPlace();
        simpleParkAtObservation();
        prepareForTeleOp();
        reset();
        //这上面的代码临时的

//        waitForStart();
////        sleep(5000);
//        blueHighChamberAim(0);
//        moveToBlueChamberPlace(0);
//        highChamberPlace();
//



////        reset();
////        reset();
//        resetAfterHighChamberAndMoveToIntakeFirst();
//        intakeBlueSample();
//        reset();
//
//        placeBlueSampleAtHP();
//        clawBlueSampleUp();
//
//        blueHighChamberAim(3);
//        moveToBlueChamberPlace(3);
//        highChamberPlace();
//
////        reset();
//        resetAfterHighChamberAndMoveToIntakeSecond();
//
//        intakeBlueSample();
//
//        reset();
//
//        placeBlueSampleAtHP();
//        clawBlueSampleUp();
//
//        blueHighChamberAim(6);
//        moveToBlueChamberPlace(6);
//        highChamberPlace();
//
//        reset();


//        pushTwoBlueSamples();
//        moveAndIntakeLastBasketSampleBlue();
//        reset();
//        placeLastBlueSampleAtHP();
//
//        clawBlueSampleUp();
//        highChamberAim();
//        highChamberPlace();
//
//        reset();
//
//        clawIntakePlace();
//        clawBlueSampleUp();
//        highChamberAim();
//        moveToBlueChamberPlace();
//        highChamberPlace();
//
//        reset();

//        prepareToClawIntakeBlueSpecimen(-4);
//
//        clawBlueSampleUp();
//        moveToBlueChamberAim();
//        highChamberAim();
//        moveToBlueChamberPlace(8);
//        highChamberPlace();
//        prepareToClawIntakeBlueSpecimen(-7);
//
//        reset();
    }


}
