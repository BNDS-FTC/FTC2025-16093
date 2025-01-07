package org.firstinspires.ftc.teamcode.testings;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name="simple drive")
public class SimpleDrive extends LinearOpMode {

    private DcMotor mLeftFront = null;
    private DcMotor mLeftBack = null;
    private DcMotor mRightFront = null;
    private DcMotor mRightBack = null;

    public void setHeadingPower(double x, double y, double rx) {
        double botHeading = 0;

        double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
        double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

        rotX = rotX * 1.1;

        y = y;
        x = x;
        rx = rx;

        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
        double frontLeftPower = (rotY + rotX + rx) / denominator;
        double backLeftPower = (rotY - rotX + rx) / denominator;
        double frontRightPower = (rotY - rotX - rx) / denominator;
        double backRightPower = (rotY + rotX - rx) / denominator;

        mLeftFront.setPower(frontLeftPower);
        mLeftBack.setPower(backLeftPower);
        mRightFront.setPower(frontRightPower);
        mRightBack.setPower(backRightPower);
    }

    @Override
    public void runOpMode() {
        mLeftFront = hardwareMap.get(DcMotor.class, "leftFront");
        mLeftBack = hardwareMap.get(DcMotor.class, "leftBack");
        mRightFront = hardwareMap.get(DcMotor.class, "rightFront");
        mRightBack = hardwareMap.get(DcMotor.class, "rightBack");

        mRightBack.setDirection(DcMotorSimple.Direction.FORWARD);
        mRightFront.setDirection(DcMotorSimple.Direction.FORWARD);
        mLeftBack.setDirection(DcMotorSimple.Direction.REVERSE);
        mLeftFront.setDirection(DcMotorSimple.Direction.REVERSE);


        waitForStart();
        while (opModeIsActive()) {
            setHeadingPower(gamepad1.left_stick_x,gamepad1.left_stick_y,gamepad1.right_stick_x);
        }
    }
}
