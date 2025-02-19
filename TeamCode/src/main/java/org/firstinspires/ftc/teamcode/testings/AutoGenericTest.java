package org.firstinspires.ftc.teamcode.testings;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.opmodes.auto.AutoMaster;


@Autonomous(name =  "Generic Auto Test", group = "Testing")
public class AutoGenericTest extends AutoMaster {
    @Override
    public void runOpMode() throws InterruptedException {
        initAuto(new Pose2d(52.3, 55, Math.toRadians(-135)));

        while(opModeInInit()){

        }

        waitForStart();

        autoResetArmTest();
        delay(50000);

    }
}