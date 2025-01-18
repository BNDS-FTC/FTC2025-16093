package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "16093 Blue Basket")
public class TeleOpBlueBasket extends TeleOp16093{
    @Override
    public void runOpMode() throws InterruptedException {
        initTeleOp(new Pose2d(0,0,Math.toRadians(90)));
        waitForStart();
        teleOpLoop();
    }
}
