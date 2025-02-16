package org.firstinspires.ftc.teamcode.drive;

import static org.firstinspires.ftc.teamcode.drive.DriveConstants.MAX_ACCEL;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.MAX_ANG_ACCEL;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.MAX_ANG_VEL;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.MAX_VEL;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.MOTOR_VELO_PID;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.RUN_USING_ENCODER;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.TRACK_WIDTH;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.kA;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.kStatic;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.kV;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.acmerobotics.roadrunner.drive.DriveSignal;
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
import org.firstinspires.ftc.teamcode.gobildapinpoint.GoBildaPinpointDriver;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequenceBuilder;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequenceRunner;
import org.firstinspires.ftc.teamcode.SuperStructure;
import org.firstinspires.ftc.teamcode.util.LynxModuleUtil;
import org.firstinspires.ftc.teamcode.util.SlewRateLimiter;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.function.BooleanSupplier;

import XCYOS.Task;

/*
 * Simple mecanum drive hardware implementation for REV hardware.
 */
@Config
public class NewMecanumDrive extends MecanumDrive {
    public static PIDCoefficients TRANS_PID = new PIDCoefficients(10, 0, 0);
    public static PIDCoefficients HEADING_PID = new PIDCoefficients(8, 0.001, 1); //i = 0

    public static double LATERAL_MULTIPLIER = 1;

    public static double VX_WEIGHT = 1;
    public static double VY_WEIGHT = 1;
    public static double OMEGA_WEIGHT = 1;

    private TrajectorySequenceRunner trajectorySequenceRunner;

    private static final TrajectoryVelocityConstraint VEL_CONSTRAINT = getVelocityConstraint(MAX_VEL, MAX_ANG_VEL, TRACK_WIDTH);
    private static final TrajectoryAccelerationConstraint ACCEL_CONSTRAINT = getAccelerationConstraint(MAX_ACCEL);

    private TrajectoryFollower follower;
    public DcMotorEx leftFront, leftRear, rightRear, rightFront;

    private List<DcMotorEx> motors;
    private GoBildaPinpointDriver odo;
    private VoltageSensor batteryVoltageSensor;

    private List<Integer> lastEncPositions = new ArrayList<>();
    private List<Integer> lastEncVels = new ArrayList<>();
    private Runnable updateRunnable;
    SlewRateLimiter driveLimiter;
    SlewRateLimiter turnLimiter;
    SlewRateLimiter slideUpDriveLimiter;

    boolean manualSwitchDrive = false;

    private double yawHeading = 0;
    public void setUpdateRunnable(Runnable updateRunnable) {
        this.updateRunnable = updateRunnable;
    }
    private BooleanSupplier switchDrivePIDCondition = ()->false;
    public void setSwitchDrivePIDCondition(BooleanSupplier switchDrivePIDCondition) {
        this.switchDrivePIDCondition = switchDrivePIDCondition;
    }
    private boolean switchDrive = false;

    public NewMecanumDrive(HardwareMap hardwareMap) {
        super(kV, kA, kStatic, TRACK_WIDTH, TRACK_WIDTH, LATERAL_MULTIPLIER);

        follower = new HolonomicPIDVAFollower(TRANS_PID, TRANS_PID, HEADING_PID,
                new Pose2d(0.5, 0.5, Math.toRadians(5.0)), 0.5);

        LynxModuleUtil.ensureMinimumFirmwareVersion(hardwareMap);

        batteryVoltageSensor = hardwareMap.voltageSensor.iterator().next();

        odo = hardwareMap.get(GoBildaPinpointDriver.class,"odo");
        odo.setOffsets(-45,115);
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_SWINGARM_POD);
        odo.recalibrateIMU();

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

        if (RUN_USING_ENCODER) {
            setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        if (RUN_USING_ENCODER && MOTOR_VELO_PID != null) {
            setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, MOTOR_VELO_PID);
        }

        rightFront.setDirection(DcMotorSimple.Direction.FORWARD);
        rightRear.setDirection(DcMotorSimple.Direction.FORWARD);
        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftRear.setDirection(DcMotorSimple.Direction.REVERSE);

        List<Integer> lastTrackingEncPositions = new ArrayList<>();
        List<Integer> lastTrackingEncVels = new ArrayList<>();

        setLocalizer(new StandardLocalizer(hardwareMap));

        trajectorySequenceRunner = new TrajectorySequenceRunner(
                follower, HEADING_PID, batteryVoltageSensor,
                lastEncPositions, lastEncVels, lastTrackingEncPositions, lastTrackingEncVels
        );

        driveLimiter = new SlewRateLimiter(6);
        turnLimiter = new SlewRateLimiter(4);
        slideUpDriveLimiter = new SlewRateLimiter(0.5);
        odo.recalibrateIMU();
    }

    public TrajectoryBuilder trajectoryBuilder(Pose2d startPose) {
        return new TrajectoryBuilder(startPose, VEL_CONSTRAINT, ACCEL_CONSTRAINT);
    }

    public TrajectoryBuilder trajectoryBuilder(Pose2d startPose, boolean reversed) {
        return new TrajectoryBuilder(startPose, reversed, VEL_CONSTRAINT, ACCEL_CONSTRAINT);
    }

    public TrajectoryBuilder trajectoryBuilder(Pose2d startPose, double startHeading) {
        return new TrajectoryBuilder(startPose, startHeading, VEL_CONSTRAINT, ACCEL_CONSTRAINT);
    }

    public TrajectorySequenceBuilder trajectorySequenceBuilder(Pose2d startPose) {
        return new TrajectorySequenceBuilder(
                startPose,
                VEL_CONSTRAINT, ACCEL_CONSTRAINT,
                MAX_ANG_VEL, MAX_ANG_ACCEL
        );
    }

    public void turnAsync(double angle) {
        trajectorySequenceRunner.followTrajectorySequenceAsync(
                trajectorySequenceBuilder(getPoseEstimate())
                        .turn(angle)
                        .build()
        );
    }

    public void turn(double angle) {
        turnAsync(angle);
        waitForIdle();
    }

    public void followTrajectoryAsync(Trajectory trajectory) {
        trajectorySequenceRunner.followTrajectorySequenceAsync(
                trajectorySequenceBuilder(trajectory.start())
                        .addTrajectory(trajectory)
                        .build()
        );
    }

    public void followTrajectory(Trajectory trajectory) {
        followTrajectoryAsync(trajectory);
        waitForIdle();
    }

    public void followTrajectorySequenceAsync(TrajectorySequence trajectorySequence) {
        trajectorySequenceRunner.followTrajectorySequenceAsync(trajectorySequence);
    }

    public void followTrajectorySequence(TrajectorySequence trajectorySequence) {
        followTrajectorySequenceAsync(trajectorySequence);
        waitForIdle();
    }

    public Pose2d getLastError() {
        return trajectorySequenceRunner.getLastPoseError();
    }

    public void update() {
        updatePoseEstimate();
        switchDrive = switchDrivePIDCondition.getAsBoolean()&&manualSwitchDrive;
        DriveSignal signal = trajectorySequenceRunner.update(getPoseEstimate(), getPoseVelocity());
        if (simpleMoveIsActivate) {
            simpleMovePeriod();
        } else if (signal != null) {
            setDriveSignal(signal);
        }
//        updatePoseEstimate();
//        DriveSignal signal = trajectorySequenceRunner.update(getPoseEstimate(), getPoseVelocity());
//        if (simpleMoveIsActivate) {
//            simpleMovePeriod();
//        } else if (signal != null) {
//            setDriveSignal(signal);
//        }
    }

    public void waitForIdle() {
        while (!Thread.currentThread().isInterrupted() && isBusy())
            updateRunnable.run();
    }

    public boolean isBusy() {
        if (simpleMoveIsActivate) {
            Pose2d err = getSimpleMovePosition().minus(getPoseEstimate());
            return  Math.abs(err.getX()) > simpleMove_x_Tolerance || Math.abs(err.getY()) > simpleMove_y_Tolerance || Math.abs(AngleUnit.normalizeRadians(err.getHeading())) > simpleMoveRotationTolerance;
        }
        return trajectorySequenceRunner.isBusy();
    }

    public void setMode(DcMotor.RunMode runMode) {
        for (DcMotorEx motor : motors) {
            motor.setMode(runMode);
        }
    }

    public void setZeroPowerBehavior(DcMotor.ZeroPowerBehavior zeroPowerBehavior) {
        for (DcMotorEx motor : motors) {
            motor.setZeroPowerBehavior(zeroPowerBehavior);
        }
    }

    public void setPIDFCoefficients(DcMotor.RunMode runMode, PIDFCoefficients coefficients) {
        PIDFCoefficients compensatedCoefficients = new PIDFCoefficients(
                coefficients.p, coefficients.i, coefficients.d,
                coefficients.f * 12 / batteryVoltageSensor.getVoltage()
        );

        for (DcMotorEx motor : motors) {
            motor.setPIDFCoefficients(runMode, compensatedCoefficients);
        }
    }

    public void setWeightedDrivePower(Pose2d drivePower) {
        Pose2d vel = drivePower;

        if (Math.abs(drivePower.getX()) + Math.abs(drivePower.getY())
                + Math.abs(drivePower.getHeading()) > 1) {
            // re-normalize the powers according to the weights
            //TODO: CHANGE THE SIGN OF X AND Y
            double denom = VX_WEIGHT * Math.abs(drivePower.getX())
                    + VY_WEIGHT * Math.abs(drivePower.getY())
                    + OMEGA_WEIGHT * Math.abs(drivePower.getHeading());

            vel = new Pose2d(
                    VX_WEIGHT * drivePower.getX(),
                    VY_WEIGHT * drivePower.getY(),
                    OMEGA_WEIGHT * drivePower.getHeading()
            ).div(denom);
        }

        setDrivePower(vel);
    }

    public static boolean ignoreDriveCoefficients = false;
    public void setFieldCentric(double x, double y, double rx, SuperStructure.Sequences sequence) {
        double botHeading = getHeading();
        double driveCoefficientTrans;
        double driveCoefficientRot;


        if(sequence == SuperStructure.Sequences.INTAKE_FAR){
            driveCoefficientTrans = 0.4;
            driveCoefficientRot = 0.4;
        }else if(sequence == SuperStructure.Sequences.INTAKE_NEAR){
            driveCoefficientTrans = 0.3;
            driveCoefficientRot = 0.3;
        }else if (sequence == SuperStructure.Sequences.LOW_BASKET||sequence==SuperStructure.Sequences.HIGH_BASKET){
            driveCoefficientTrans = 0.9;
            driveCoefficientRot = 0.6;
        } else if (sequence == SuperStructure.Sequences.HIGH_CHAMBER){
            driveCoefficientRot = 0.5;
            driveCoefficientTrans = 0.5;
        }else{
            driveCoefficientTrans = 1;
            driveCoefficientRot = 1;
        }

        if(ignoreDriveCoefficients) {
            driveCoefficientTrans = 1;
            driveCoefficientRot = 1;
        }

        y = y*-driveCoefficientTrans;
        x = x*driveCoefficientTrans;
        rx =rx*-driveCoefficientRot;

        if(sequence == SuperStructure.Sequences.HIGH_BASKET){
            y = driveLimiter.calculate(y);
            rx = turnLimiter.calculate(rx);
        }

        double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
        double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);
        rotX = rotX * 1.1;

        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
        double frontLeftPower = (rotY + rotX + rx) / denominator;
        double backLeftPower = (rotY - rotX + rx) / denominator;
        double frontRightPower = (rotY - rotX - rx) / denominator;
        double backRightPower = (rotY + rotX - rx) / denominator;

        setMotorPowers(frontLeftPower, backLeftPower, backRightPower, frontRightPower);
    }

    public void setBotCentric(double x, double y, double rx, SuperStructure.Sequences sequence) {
        double botHeading = 0;
        double driveCoefficientTrans;
        double driveCoefficientRot;

        if(sequence == SuperStructure.Sequences.INTAKE_FAR){
            driveCoefficientTrans = 0.3;
            driveCoefficientRot = 0.2;
        }else if(sequence == SuperStructure.Sequences.INTAKE_NEAR){
            driveCoefficientTrans = 0.3;
            driveCoefficientRot = 0.3;
        }else if (sequence == SuperStructure.Sequences.LOW_BASKET||sequence==SuperStructure.Sequences.HIGH_BASKET){
            driveCoefficientTrans = 0.9;
            driveCoefficientRot = 0.5;
        } else if (sequence == SuperStructure.Sequences.HIGH_CHAMBER||sequence==SuperStructure.Sequences.ASCENT){
            driveCoefficientRot = 0.7;
            driveCoefficientTrans = 0.7;
        }else{
            driveCoefficientTrans = 1;
            driveCoefficientRot = 1;
        }

        if(ignoreDriveCoefficients) {
            driveCoefficientTrans = 1;
            driveCoefficientRot = 1;
        }

        y = y*-driveCoefficientTrans;
        x = x*driveCoefficientTrans;
        rx = rx*-driveCoefficientRot;

        double denominator = Math.max(Math.abs(x) + Math.abs(y) + Math.abs(rx), 1);
        double frontLeftPower = (y + x + rx) / denominator;
        double backLeftPower = (y - x + rx) / denominator;
        double frontRightPower = (y - x - rx) / denominator;
        double backRightPower = (y + x - rx) / denominator;

        setMotorPowers(frontLeftPower, backLeftPower, backRightPower, frontRightPower);
    }

    @NonNull
    @Override
    public List<Double> getWheelPositions() {
        lastEncPositions.clear();

        List<Double> wheelPositions = new ArrayList<>();
        lastEncPositions.add(odo.getEncoderX());
        wheelPositions.add(mmToInches(odo.getPosX()));

        lastEncPositions.add(odo.getEncoderY());
        wheelPositions.add(mmToInches(odo.getPosY()));

        return wheelPositions;
    }

    @Override
    public List<Double> getWheelVelocities() {
        lastEncVels.clear();

        List<Double> wheelVelocities = new ArrayList<>();

        // TODO: 2024/10/30 getRawVelocity
        lastEncVels.add((int) odo.getVelX());
        wheelVelocities.add(mmToInches(odo.getVelX()));

        lastEncVels.add((int)odo.getVelY());
        wheelVelocities.add(mmToInches(odo.getVelY()));

        return wheelVelocities;
    }

    @Override
    public void setMotorPowers(double lf, double lr, double rr, double rf) {
        leftFront.setPower(lf);
        leftRear.setPower(lr);
        rightRear.setPower(rr);
        rightFront.setPower(rf);
    }

    /*
    Array of length 4
     */
    public void setMotorPowers(double[] powers) {
        leftFront.setPower(powers[0]);
        leftRear.setPower(powers[1]);
        rightRear.setPower(powers[2]);
        rightFront.setPower(powers[3]);
    }

    @Override
    public double getRawExternalHeading() {
        return odo.getHeading();
    }

    @Override
    public Double getExternalHeadingVelocity() {
        return (double) odo.getHeadingVelocity();
    }

    public static TrajectoryVelocityConstraint getVelocityConstraint(double maxVel, double maxAngularVel, double trackWidth) {
        return new MinVelocityConstraint(Arrays.asList(
                new AngularVelocityConstraint(maxAngularVel),
                new MecanumVelocityConstraint(maxVel, trackWidth)
        ));
    }

    public static TrajectoryAccelerationConstraint getAccelerationConstraint(double maxAccel) {
        return new ProfileAccelerationConstraint(maxAccel);
    }

    public static PIDCoefficients translationXPid = new PIDCoefficients(0.1298, 0, 0.0094);
    public static PIDCoefficients translationYPid = new PIDCoefficients(0.075, 0, 0.009);
    public static PIDCoefficients headingPid = new PIDCoefficients(0.878, 0.00002, 0.05);

    private PIDFController transPID_x;
    private PIDFController transPID_y;
    private PIDFController turnPID;


    public static PIDCoefficients armUpXPid = new PIDCoefficients(0.0898, 0, 0.0094);
    public static PIDCoefficients armUpYPid = new PIDCoefficients(0.075, 0, 0.0009);
    public static PIDCoefficients armUpHeadingPid = new PIDCoefficients(0.878, 0.0002, 0.05);

    private PIDFController armUpTransPID_x;
    private PIDFController armUpTransPID_y;
    private PIDFController armUpTurnPID;


    private double moveHeading = 0;

    private static final double DEFAULT_TRANS_TOL = 1.25;

    private double simpleMove_x_Tolerance = 1.25, simpleMove_y_Tolerance = 1.25, simpleMoveRotationTolerance = Math.toRadians(10);
    private double simpleMovePower = 0.95;
    public boolean simpleMoveIsActivate = false; //private

    public void setSimpleMoveTolerance(double x, double y, double rotation) {
        simpleMove_x_Tolerance = x;
        simpleMove_y_Tolerance = y;
        simpleMoveRotationTolerance = rotation;
    }

    public void setSimpleMovePower(double power) {
        simpleMovePower = power;
    }

    public void stopTrajectory() {
        trajectorySequenceRunner.followTrajectorySequenceAsync(null);
        simpleMoveIsActivate = false;
    }

    public void initSimpleMove(Pose2d pos) {
        stopTrajectory();
        simpleMoveIsActivate = true;
        transPID_x = new PIDFController(translationXPid);
        transPID_x.setTargetPosition(pos.getX());

        transPID_y = new PIDFController(translationYPid);
        transPID_y.setTargetPosition(pos.getY());

        turnPID = new PIDFController(headingPid);
        moveHeading = pos.getHeading();
        turnPID.setTargetPosition(0);

        armUpTransPID_x = new PIDFController(armUpXPid);
        armUpTransPID_x.setTargetPosition(pos.getX());

        armUpTransPID_y = new PIDFController(armUpYPid);
        armUpTransPID_y.setTargetPosition(pos.getY());

        armUpTurnPID = new PIDFController(armUpHeadingPid);
        moveHeading = pos.getHeading();
        armUpTurnPID.setTargetPosition(0);
    }

    //    @Deprecated
    public boolean simpleMoveInDistress = false;
    public void moveTo(Pose2d endPose, int correctTime_ms) {
        long startTime = System.currentTimeMillis();
        simpleMoveInDistress = false;
        initSimpleMove(endPose);
        while (isBusy()) {
            updateRunnable.run();
//            if(System.currentTimeMillis() - startTime > 10000){
//                simpleMoveIsActivate = false;
//                setMotorPowers(0, 0, 0, 0);
//                simpleMoveInDistress = true;
//            }
        }
        long endTime = System.currentTimeMillis() + correctTime_ms;
        while (endTime > System.currentTimeMillis()) {
            updateRunnable.run();
//            if(System.currentTimeMillis() - startTime > 10000){
//                simpleMoveIsActivate = false;
//                setMotorPowers(0, 0, 0, 0);
//                simpleMoveInDistress = true;
//            }
        }
        simpleMoveIsActivate = false;
        setMotorPowers(0, 0, 0, 0);
    }

    public void moveTo(Pose2d endPose, int correctTime_ms, Runnable runWhileMoving) {
        long startTime = System.currentTimeMillis();
        simpleMoveInDistress = false;
        initSimpleMove(endPose);
        while (isBusy()) {
            updateRunnable.run();
            runWhileMoving.run();
//            if(System.currentTimeMillis() - startTime > 10000){
////                simpleMoveIsActivate = false;
//                setMotorPowers(0, 0, 0, 0);
//                simpleMoveInDistress = true;
//            }
        }
        long endTime = System.currentTimeMillis() + correctTime_ms;
        while (endTime > System.currentTimeMillis()) {
            updateRunnable.run();
            runWhileMoving.run();
//            if(System.currentTimeMillis() - startTime > 10000){
////                simpleMoveIsActivate = false;
//                setMotorPowers(0, 0, 0, 0);
//                simpleMoveInDistress = true;
//            }
        }
        simpleMoveIsActivate = false;
        setMotorPowers(0, 0, 0, 0);
    }

    /*
    startPose and endPose must have (same x && !same y) || (same y && !same x).
    Also this only works for four directions because I am bad at geometry. --Annie
     */
    private double turnWheelSpeedDiff = 0;
    private double poseDiff;
    private double[] motorPowerStorage = new double[4];
    private turnDirection direction;
    public void calculateSemicircles(Pose2d startPose, Pose2d endPose){
        if(endPose.getY() == startPose.getY()){
            poseDiff = endPose.getX() - startPose.getX();
            //Difference is calculated based on x direction.
            turnWheelSpeedDiff = calculateWheelSpeedDiff(poseDiff);
            if(Math.toDegrees(startPose.getHeading()) == 0){
                direction = poseDiff>0? turnDirection.LEFT:turnDirection.RIGHT;
            }else if(Math.toDegrees(startPose.getHeading()) == 180){
                direction = poseDiff>0? turnDirection.RIGHT:turnDirection.LEFT;
            }
        }else{
            poseDiff = endPose.getY() - startPose.getY();
            //Difference is calculated based on y direction.
            turnWheelSpeedDiff = calculateWheelSpeedDiff(poseDiff);
            if(Math.toDegrees(startPose.getHeading()) == 90){
                direction = poseDiff>0? turnDirection.RIGHT:turnDirection.LEFT;
            }else if(Math.toDegrees(startPose.getHeading()) == 270) {
                direction = poseDiff > 0 ? turnDirection.LEFT : turnDirection.RIGHT;
            }
        }

        if(endPose.getHeading() > startPose.getHeading()){
            stopSemicircling = odo.getHeading() > endPose.getHeading();
        }else{
            stopSemicircling = odo.getHeading() < endPose.getHeading();
        }
    }
    public double[] setStitchSemicirclePower(){
        if(direction == turnDirection.LEFT){
            motorPowerStorage[0] = 1-turnWheelSpeedDiff;
            motorPowerStorage[1] = 1-turnWheelSpeedDiff;
            motorPowerStorage[2] = 1;
            motorPowerStorage[3] = 1;
        }else{
            motorPowerStorage[0] = 1;
            motorPowerStorage[1] = 1;
            motorPowerStorage[2] = 1-turnWheelSpeedDiff;
            motorPowerStorage[3] = 1-turnWheelSpeedDiff;
        }
        return motorPowerStorage;
    }

    public boolean stopSemicircling = true;
    public void stitchSemicircleTo(Pose2d startPose, Pose2d endPose, int correctTime_ms){
        long startTime = System.currentTimeMillis();
        simpleMoveInDistress = false;
        stopSemicircling = false;
        setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        calculateSemicircles(startPose, endPose);
        initSimpleMove(endPose);
        while (isBusy()) {
            updateRunnable.run();
            while(!stopSemicircling){
                setMotorPowers(setStitchSemicirclePower());
                updateRunnable.run();
            }
        }
        long endTime = System.currentTimeMillis() + correctTime_ms;
        while (endTime > System.currentTimeMillis()) {
            updateRunnable.run();
        }
        simpleMoveIsActivate = false;
        setMotorPowers(0, 0, 0, 0);
        setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    }

    //TODO: MATH!
    public static double semicircleCoefficient = 1.1;
    private double calculateWheelSpeedDiff(double poseDiff){
        //Apply math
        return 1.2;
    }
    private enum turnDirection{
        LEFT, RIGHT
    }

    public void moveWithNoBrake(Pose2d...poses){
        setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        for(Pose2d p:poses){
            moveTo(p,0);
        }
        setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void moveWithoutStopping(Pose2d endPose, int correctTime_ms) {
        long startTime = System.currentTimeMillis();
        simpleMoveInDistress = false;
        initSimpleMove(endPose);
        while (isBusy()) {
            updateRunnable.run();
        }
        long endTime = System.currentTimeMillis() + correctTime_ms;
        while (endTime > System.currentTimeMillis()) {
            updateRunnable.run();
        }
        simpleMoveIsActivate = false;
//        setMotorPowers(0, 0, 0, 0);
    }

    public Pose2d getSimpleMovePosition() {
        return new Pose2d(transPID_x.getTargetPosition(), transPID_y.getTargetPosition(), moveHeading);
    }

    public Task simpleMoveTime(Pose2d pose, int time, double power, double toleranceRate) {
        return new Task() {
            private long endTime;
            private boolean atReqPos;

            @Override
            public void setUp() {
                atReqPos = toleranceRate<0;
                initSimpleMove(pose);
                simpleMovePower = power;
                simpleMove_x_Tolerance = DEFAULT_TRANS_TOL * toleranceRate;
                simpleMove_y_Tolerance = DEFAULT_TRANS_TOL * toleranceRate;
                endTime = System.currentTimeMillis() + time;
                simpleMoveIsActivate = true;
            }

            @Override
            public void run() {
                if (!atReqPos) {
                    atReqPos = isBusy();
                } else if (endTime < System.currentTimeMillis()) {
                    status = Status.ENDED;
                }
            }

            @Override
            public void end() {
                simpleMoveIsActivate = false;
                simpleMove_x_Tolerance = DEFAULT_TRANS_TOL * toleranceRate;
                simpleMove_y_Tolerance = DEFAULT_TRANS_TOL * toleranceRate;
            }
        };
    }

    public Task simpleMoveTime(Pose2d pose, int time) {
        return simpleMoveTime(pose, time, 0.5, 1);
    }

    public static final double DEAD_BAND = 0.0001;

    public void moveWithDrift(Pose2d... poses) {
        setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT); // 关闭刹车，允许自由漂移

        for (Pose2d targetPose : poses) {
            moveTo(targetPose,0); // 对每个目标点调用带漂移的移动方法
        }

        setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE); // 恢复刹车行为
    }

    // 移动到目标点并在接近时停止施加功率
    public void moveToWithDrift(Pose2d targetPose) {
        double driftThreshold = 2;

        while (true) {
            // 获取当前机器人位置
            Pose2d currentPose = getPoseEstimate();
            double distanceToTarget = calculateDistance(currentPose, targetPose); // 计算到目标点的距离

            if (distanceToTarget <= driftThreshold) {
                setMotorPowers(0, 0, 0, 0); // 停止电机功率，让机器人漂移
                break;
            }

            // 根据距离计算速度，逐渐减小速度
            double speed = calculateSpeed(distanceToTarget);
            double motorPower = speed;

            // 设置电机功率
            setMotorPowers(motorPower, motorPower, motorPower, motorPower);
        }
    }

    // 计算当前点与目标点之间的距离
    private double calculateDistance(Pose2d currentPose, Pose2d targetPose) {
        double dx = targetPose.getX() - currentPose.getX();
        double dy = targetPose.getY() - currentPose.getY();
        return Math.sqrt(dx * dx + dy * dy); // 使用欧几里得距离
    }

    // 根据距离计算速度
    private double calculateSpeed(double distanceToTarget) {
        double maxSpeed = 1.0; // 最大速度
        double minSpeed = 0.1; // 最小速度（可调整，确保机器人慢速接近）

        // 距离越近，速度越小（线性比例缩放）
        double speed = maxSpeed * (distanceToTarget / 3.5);
        return Math.min(speed, maxSpeed); // 确保速度不大于最大值
    }

    // 将速度转换为电机功率
    private double speedToMotorPower(double speed) {
        return speed; // 假设速度范围与功率范围一致
    }


    /**
     * 无头功率
     *
     * @param drivePower
     * @param x_static
     * @param y_static
     */
    public void setGlobalPower(Pose2d drivePower, double x_static, double y_static) {
        Vector2d vec = drivePower.vec().rotated(-getLocalizer().getPoseEstimate().getHeading());
//        Vector2d vec = drivePower.vec().rotated(-getRawExternalHeading());
        if (vec.norm() > DEAD_BAND) {
            vec = new Vector2d(
                    vec.getX() + Math.copySign(x_static, vec.getX()),
                    vec.getY() + Math.copySign(y_static, vec.getY())
            );
        }
        setWeightedDrivePower(new Pose2d(vec, drivePower.getHeading()));
    }

    public void simpleMovePeriod() {
        Pose2d current_pos = getPoseEstimate();
        if(switchDrive){
            this.setGlobalPower(new Pose2d(
                    clamp(armUpTransPID_x.update(current_pos.getX()), simpleMovePower),
                    clamp(armUpTransPID_y.update(current_pos.getY()), simpleMovePower),
                    clamp(armUpTurnPID.update(AngleUnit.normalizeRadians(current_pos.getHeading() - moveHeading)), simpleMovePower)
            ), 0, 0);
        }else{
            this.setGlobalPower(new Pose2d(
                    clamp(transPID_x.update(current_pos.getX()), simpleMovePower),
                    clamp(transPID_y.update(current_pos.getY()), simpleMovePower),
                    clamp(turnPID.update(AngleUnit.normalizeRadians(current_pos.getHeading() - moveHeading)), simpleMovePower)
            ), 0, 0);
        }
    }


    public Task updatePositionTask = new Task() {
        @Override
        public void run() {
            update();
        }
    };

    private double clamp(double val, double range) {
        return Range.clip(val, -range, range);
    }
    public static double mmToInches(double mm) {
        return mm/25.4;
    }
    public void updateOdo(){
        odo.update();
    }

    public double getHeading() {
        return odo.getHeading() - yawHeading;
    }

    public void resetHeading(){
        yawHeading = odo.getHeading();
    }

    public void resetOdo(){
        odo.recalibrateIMU();
    }
//    public double getHeading(){
//        Pose2D pos = odo.getPosition();
//        return pos.getHeading(AngleUnit.DEGREES);
//    }

    public Pose2d lastStoredPos;
    public void storeCurrentPos(){
        if(!simpleMoveIsActivate){
            lastStoredPos = odo.getPositionAsPose2d();
        }
    }
    public String getStoredPosAsString(){
        if(lastStoredPos != null){
            return lastStoredPos.toString();
        }
        return "POSE NOT PROPERLY INITIALIZED!!!!!";
    }

    public String getCurrentPoseAsString(){
        return odo.getPositionAsPose2d().toString();
    }
    public Pose2d getCurrentPose(){
        return odo.getPositionAsPose2d();
    }

    public void testSemicircleRadius(double speedDiff){
        setMotorPowers(1-speedDiff,1-speedDiff,1,1);
    }

    public void turnOnSwitchDrive(boolean on){
        manualSwitchDrive = on;
    }


    public String printMotorSpeeds(){
        String ret = "";
        for(DcMotorEx motor:motors){
            ret += (motor.getCurrentPosition())+" ";
        }
        return ret;
    }
}