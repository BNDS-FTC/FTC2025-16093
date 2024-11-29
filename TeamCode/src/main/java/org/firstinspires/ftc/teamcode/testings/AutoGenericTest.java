package org.firstinspires.ftc.teamcode.testings;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.AutoMaster;

@Autonomous(name =  "Generic Auto Test", group = "Testing")
public class AutoGenericTest extends AutoMaster {
    @Override
    public void runOpMode() throws InterruptedException {
        side_color = BLUE;
        startSide = NEGATIVE;

        // TODO: THIS IS BROKEN!
        initHardware(new Pose2d(-15  ,62.3 ,Math.toRadians(90)));

        while(opModeInInit()){

        }

        waitForStart();

        autoUpperTest();

//        autoResetArmTest();

//        simplePushSample1Blue();
//
//        autoUpperTest();

//        highChamberPlace();
//
//        reset();
//
//        moveToPushSample1();
//
//        pushSample();

    }
}