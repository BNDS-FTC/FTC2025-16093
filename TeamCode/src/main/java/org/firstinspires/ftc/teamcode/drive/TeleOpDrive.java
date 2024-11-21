package org.firstinspires.ftc.teamcode.drive;

import static org.firstinspires.ftc.teamcode.drive.DriveConstants.MAX_ACCEL;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.MAX_ANG_ACCEL;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.MAX_ANG_VEL;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.MAX_VEL;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.MOTOR_VELO_PID;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.RUN_USING_ENCODER;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.TRACK_WIDTH;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.encoderTicksToInches;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.kA;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.kStatic;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.kV;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.acmerobotics.roadrunner.drive.MecanumDrive;
import com.acmerobotics.roadrunner.followers.HolonomicPIDVAFollower;
import com.acmerobotics.roadrunner.followers.TrajectoryFollower;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.acmerobotics.roadrunner.trajectory.constraints.AngularVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MinVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.ProfileAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.gobildapinpoint.GoBildaPinpointDriver;
import org.firstinspires.ftc.teamcode.uppersystems.SuperStructure;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequenceBuilder;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequenceRunner;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.List;

import XCYOS.Component;
import XCYOS.Task;
import XCYOS.TaskChainBuilder;

/*
 * Simple mecanum drive hardware implementation for REV hardware.
 */
@Config
public class TeleOpDrive{

    public static double LATERAL_MULTIPLIER = 1;

    public static double VX_WEIGHT = 1;
    public static double VY_WEIGHT = 1;
    public static double OMEGA_WEIGHT = 1;

    private TrajectorySequenceRunner trajectorySequenceRunner;
    private DcMotorEx leftFront, leftRear, rightRear, rightFront;

    private List<DcMotorEx> motors;
    GoBildaPinpointDriver odo;
    private VoltageSensor batteryVoltageSensor;

    public void setUp(HardwareMap hardwareMap) {
        odo = hardwareMap.get(GoBildaPinpointDriver.class,"odo");
        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        leftRear = hardwareMap.get(DcMotorEx.class, "leftBack");
        rightRear = hardwareMap.get(DcMotorEx.class, "rightBack");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");
        motors = Arrays.asList(leftFront, leftRear, rightRear, rightFront);
        for (DcMotorEx motor : motors) {
            MotorConfigurationType motorConfigurationType = motor.getMotorType().clone();
            motorConfigurationType.setAchieveableMaxRPMFraction(1.0);
            motor.setMotorType(motorConfigurationType);
        }

        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftRear.setDirection(DcMotorSimple.Direction.REVERSE);
        rightFront.setDirection(DcMotorSimple.Direction.FORWARD);
        rightRear.setDirection(DcMotorSimple.Direction.FORWARD);

        if (RUN_USING_ENCODER) {
            setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        //setZeroPowerBehavior(true);

        batteryVoltageSensor = hardwareMap.voltageSensor.iterator().next();

        odo.setOffsets(-100, 110); //these are tuned for 3110-0002-0001 Product Insight #1

        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD);
        odo.recalibrateIMU();
        odo.resetPosAndIMU();
    }


    public double getYaw() {
        return odo.getHeading();
    }

    //THIS IS WRONG AND TEMPORARY!!!!!!! THIS MUST BE CHANGED LATER!!!!!!!
    public void setHeadingPower(double x, double y, double rx, SuperStructure.Sequences sequence) {
        double botHeading = 0;
        double driveCoefficientTrans;
        double driveCoefficientRot;


        double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
        double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

        rotX = rotX * 1.1;

        if(sequence == SuperStructure.Sequences.INTAKE_FAR || sequence == SuperStructure.Sequences.HIGH_BASKET){
            driveCoefficientTrans = 0.04;
            driveCoefficientRot = 0.04;
        }else if(sequence == SuperStructure.Sequences.INTAKE_NEAR){
            driveCoefficientTrans = 0.05;
            driveCoefficientRot = 0.05;
        }else{
            driveCoefficientTrans = 0.3;
            driveCoefficientRot = 0.3;
        }

        y = y*-driveCoefficientTrans;
        x = x*driveCoefficientTrans;
        rx = rx*-driveCoefficientRot;

        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
        double frontLeftPower = (rotY + rotX + rx) / denominator;
        double backLeftPower = (rotY - rotX + rx) / denominator;
        double frontRightPower = (rotY - rotX - rx) / denominator;
        double backRightPower = (rotY + rotX - rx) / denominator;

        leftFront.setPower(frontLeftPower);
        leftRear.setPower(backLeftPower);
        rightFront.setPower(frontRightPower);
        rightRear.setPower(backRightPower);
    }

    //THIS IS WRONG AND TEMPORARY!!!!!!! THIS MUST BE CHANGED LATER!!!!!!!
    public void setGlobalPower(double x, double y, double rx, SuperStructure.Sequences sequence) {

        double rotX = x * Math.cos(-odo.getHeading()) - y * Math.sin(-odo.getHeading());
        double rotY = x * Math.sin(-odo.getHeading()) + y * Math.cos(-odo.getHeading());

        double driveCoefficient;

        if(sequence == SuperStructure.Sequences.INTAKE_FAR || sequence == SuperStructure.Sequences.HIGH_BASKET || sequence == SuperStructure.Sequences.CUSTOM_INTAKE || sequence == SuperStructure.Sequences.HIGH_CHAMBER){
            driveCoefficient = 0.1;
        }else if(sequence == SuperStructure.Sequences.INTAKE_NEAR){
            driveCoefficient = 0.2;
        }else{
            driveCoefficient = 0.4;
        }

        rotY = rotY*-driveCoefficient;
        rotX = -rotX*driveCoefficient;
        rx = -rx*(0.7*driveCoefficient);

        setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        double frontLeftPower = (rotY + rotX + rx);// denominator;
        double backLeftPower = (rotY - rotX + rx); // denominator;
        double frontRightPower = (rotY - rotX - rx); // denominator;
        double backRightPower = (rotY + rotX - rx); // denominator;

        leftFront.setPower(frontLeftPower);
        leftRear.setPower(backLeftPower);
        rightFront.setPower(frontRightPower);
        rightRear.setPower(backRightPower);
    }

    public void setMode(DcMotor.RunMode runMode) {
        for (DcMotorEx motor : motors) {
            motor.setMode(runMode);
        }
    }


    public void resetOdo(){
        odo.resetPosAndIMU();
    }

    public double getHeading(){
        Pose2D pos = odo.getPosition();
        return pos.getHeading(AngleUnit.DEGREES);
    }

    public void updateOdo(){
        odo.update();
    }

    private double clamp(double val, double range) {
        return Range.clip(val, -range, range);
    }
}