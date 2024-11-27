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
import org.firstinspires.ftc.teamcode.util.LynxModuleUtil;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import XCYOS.Task;

/*
 * Simple mecanum drive hardware implementation for REV hardware.
 */
@Config
public class BarkMecanumDrive extends MecanumDrive {
    public static PIDCoefficients TRANSLATIONAL_PID = new PIDCoefficients(10, 0, 0);
    public static PIDCoefficients HEADING_PID = new PIDCoefficients(8, 0, 1); //8

    public static double LATERAL_MULTIPLIER = 1;

    public static double VX_WEIGHT = 1;
    public static double VY_WEIGHT = 1;
    public static double OMEGA_WEIGHT = 1;

    private TrajectorySequenceRunner trajectorySequenceRunner;

    private static final TrajectoryVelocityConstraint VEL_CONSTRAINT = getVelocityConstraint(MAX_VEL, MAX_ANG_VEL, TRACK_WIDTH);
    private static final TrajectoryAccelerationConstraint ACCEL_CONSTRAINT = getAccelerationConstraint(MAX_ACCEL);

    private TrajectoryFollower follower;
    private DcMotorEx leftFront, leftRear, rightRear, rightFront;

    private List<DcMotorEx> motors;
    private GoBildaPinpointDriver odo;
    private VoltageSensor batteryVoltageSensor;

    private List<Integer> lastEncPositions = new ArrayList<>();
    private List<Integer> lastEncVels = new ArrayList<>();
    private Runnable updateRunnable;
    public void setUpdateRunnable(Runnable updateRunnable) {
        this.updateRunnable = updateRunnable;
    }
    public BarkMecanumDrive(HardwareMap hardwareMap) {
        super(kV, kA, kStatic, TRACK_WIDTH, TRACK_WIDTH, LATERAL_MULTIPLIER);

        follower = new HolonomicPIDVAFollower(TRANSLATIONAL_PID, TRANSLATIONAL_PID, HEADING_PID,
                new Pose2d(0.5, 0.5, Math.toRadians(5.0)), 0.5);

        LynxModuleUtil.ensureMinimumFirmwareVersion(hardwareMap);

        batteryVoltageSensor = hardwareMap.voltageSensor.iterator().next();

        // TODO: adjust the names of the following hardware devices to match your configuration
        odo = hardwareMap.get(GoBildaPinpointDriver.class,"odo");
        odo.setOffsets(-100,-115);
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD);
//        odo.resetPosAndIMU();

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

        // TODO: reverse any motors using DcMotor.setDirection()
        leftFront.setDirection(DcMotorSimple.Direction.FORWARD);
        leftRear.setDirection(DcMotorSimple.Direction.FORWARD);
        rightFront.setDirection(DcMotorSimple.Direction.REVERSE);
        rightRear.setDirection(DcMotorSimple.Direction.REVERSE);

        List<Integer> lastTrackingEncPositions = new ArrayList<>();
        List<Integer> lastTrackingEncVels = new ArrayList<>();

        // TODO: if desired, use setLocalizer() to change the localization method
        setLocalizer(new StandardLocalizer(hardwareMap));

        trajectorySequenceRunner = new TrajectorySequenceRunner(
                follower, HEADING_PID, batteryVoltageSensor,
                lastEncPositions, lastEncVels, lastTrackingEncPositions, lastTrackingEncVels
        );
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
        DriveSignal signal = trajectorySequenceRunner.update(getPoseEstimate(), getPoseVelocity());
        if (simpleMoveIsActivate) {
            simpleMovePeriod();
        } else if (signal != null) {
            setDriveSignal(signal);
        }
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

    public double getYaw() {
        return odo.getHeading();
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

    public void setGlobalPower(double x, double y, double rx) {
        double botHeading = odo.getHeading();
        double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
        double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

        rotX = rotX * 1.1;

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

    public void resetHeading(){
        odo.recalibrateIMU();
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
    public void setMotorPowers(double v, double v1, double v2, double v3) {
        leftFront.setPower(v);
        leftRear.setPower(v1);
        rightRear.setPower(v2);
        rightFront.setPower(v3);
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

    public static PIDCoefficients translationPid = new PIDCoefficients(0.1778, 0.000, 0.02286);
    public static PIDCoefficients headingPid = new PIDCoefficients(1.5, 0, 0.2);

    private PIDFController transPID_x;
    private PIDFController transPID_y;
    private PIDFController turnPID;
    private double moveHeading = 0;

    private static final double DEFAULT_TRANS_TOL = 1.25;

    private double simpleMove_x_Tolerance = 1.25, simpleMove_y_Tolerance = 1.25, simpleMoveRotationTolerance = Math.toRadians(10);
    private double simpleMovePower = 0.95;
    private boolean simpleMoveIsActivate = false;

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
        transPID_x = new PIDFController(translationPid);
        transPID_x.setTargetPosition(pos.getX());

        transPID_y = new PIDFController(translationPid);
        transPID_y.setTargetPosition(pos.getY());

        turnPID = new PIDFController(headingPid);
        moveHeading = pos.getHeading();
        turnPID.setTargetPosition(0);
    }

    //    @Deprecated
    public void moveTo(Pose2d endPose, int correctTime_ms) {
        initSimpleMove(endPose);
        while (isBusy())
            updateRunnable.run();
        long endTime = System.currentTimeMillis() + correctTime_ms;
        while (endTime > System.currentTimeMillis())
            updateRunnable.run();
        simpleMoveIsActivate = false;
        setMotorPowers(0, 0, 0, 0);
    }

    private Pose2d getSimpleMovePosition() {
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
        this.setGlobalPower(new Pose2d(
                clamp(transPID_x.update(current_pos.getX()), simpleMovePower),
                clamp(transPID_y.update(current_pos.getY()), simpleMovePower),
                clamp(turnPID.update(AngleUnit.normalizeRadians(current_pos.getHeading() - moveHeading)), simpleMovePower)
        ), 0, 0);
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
}