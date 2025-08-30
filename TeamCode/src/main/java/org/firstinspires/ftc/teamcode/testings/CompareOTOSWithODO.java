/*
    SPDX-License-Identifier: MIT

    Copyright (c) 2024 SparkFun Electronics
*/
package org.firstinspires.ftc.teamcode.testings;

import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.drive.NewMecanumDrive;

@TeleOp(name = "SparkFun OTOS and Gobilda pinpoint", group = "Sensor")
public class CompareOTOSWithODO extends LinearOpMode {
    SparkFunOTOS myOtos;
    NewMecanumDrive drive;

    @Override
    public void runOpMode() throws InterruptedException {
        myOtos = hardwareMap.get(SparkFunOTOS.class, "sensor_otos");
        configureOtos();
        drive = new NewMecanumDrive(hardwareMap);
        drive.odo.setPosition(new Pose2D(DistanceUnit.INCH, 0,0,AngleUnit.DEGREES,0));
        waitForStart();
        while (opModeIsActive()) {
            drive.setBotCentric(gamepad1.left_stick_x,gamepad1.left_stick_y,gamepad1.right_stick_x,null);
            SparkFunOTOS.Pose2D pos_otos = myOtos.getPosition();
            if (gamepad1.y) {
                myOtos.resetTracking();
                drive.resetOdo();
            }
            if (gamepad1.x) {
                myOtos.calibrateImu();
            }
            telemetry.addLine("Press Y (triangle) on Gamepad to reset tracking");
            telemetry.addLine("Press X (square) on Gamepad to calibrate the IMU");
            telemetry.addLine();

            telemetry.addData("OTOS: X coordinate", pos_otos.x);
            telemetry.addData("OTOS: Y coordinate", pos_otos.y);
            telemetry.addData("OTOS: Heading angle", pos_otos.h);

            telemetry.addLine("-----");
            telemetry.addData("Pinpoint: X coordinate", drive.getCurrentPose().getX());
            telemetry.addData("Pinpoint: Y coordinate", drive.getCurrentPose().getY());
            telemetry.addData("Pinpoint: Heading angle", drive.getHeading());
            telemetry.update();
            drive.update();
            drive.updateOdo();
        }
    }

    private void configureOtos() {
        myOtos.setLinearUnit(DistanceUnit.INCH);
        myOtos.setAngularUnit(AngleUnit.DEGREES);
        SparkFunOTOS.Pose2D offset = new SparkFunOTOS.Pose2D(0, 0, 0);
        myOtos.setOffset(offset);
        myOtos.setLinearScalar(1.0);
        myOtos.setAngularScalar(1.0);
        myOtos.calibrateImu();
        myOtos.resetTracking();
        SparkFunOTOS.Pose2D currentPosition = new SparkFunOTOS.Pose2D(0, 0, 0);
        myOtos.setPosition(currentPosition);

        SparkFunOTOS.Version hwVersion = new SparkFunOTOS.Version();
        SparkFunOTOS.Version fwVersion = new SparkFunOTOS.Version();
        myOtos.getVersionInfo(hwVersion, fwVersion);}
}