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
        //   warning:Untested,这个代码是我rollback来的，我不知道它的文件有没有完全rollback成功！！！！！！！！！！
        //   warning:Untested,这个代码是我rollback来的，我不知道它的文件有没有完全rollback成功！！！！！！！！！！
        //   warning:Untested,这个代码是我rollback来的，我不知道它的文件有没有完全rollback成功！！！！！！！！！！
        waitForStart();
        highChamber(0);

        prepareForPushTwoBlueSample();
        pushTwoBlueSamples();
        reset();
        prepareForClwBlueSampleUp();
        clawBlueSampleUp();
        highChamber(3);
        //reset();
        prepareForClwBlueSampleUp();
        clawBlueSampleUp();
        highChamber(6);
        //reset();


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
