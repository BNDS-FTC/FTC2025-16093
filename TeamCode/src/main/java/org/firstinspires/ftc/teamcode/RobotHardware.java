package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.teamcode.references.Globals;
import org.firstinspires.ftc.teamcode.references.ServoPWMControl;
import org.firstinspires.ftc.teamcode.references.XCYBoolean;
import org.firstinspires.ftc.teamcode.subsystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ClawSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.GrabSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.SlideSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.WristSubsystem;

import org.firstinspires.ftc.robotcore.external.Telemetry;


public class RobotHardware {
    private DcMotorEx mArm;
    private DcMotorEx mSlideRight;
    private DcMotorEx mSlideLeft;

    private Servo mIntakeLeft; // continuous
    private Servo mIntakeRight;// continuous
    private Servo Wrist;
    private Servo Grab;
    private Servo clawLeft;
    private Servo clawRight;


    private TouchSensor mTouchSensor;

    public static PIDCoefficients armPidConf = new PIDCoefficients(0.09, 0, 0);
    private PIDFController armPidCtrl;

    public static PIDCoefficients rSlidePidConf = new PIDCoefficients(0.025, 0, 0);
    private PIDFController rSlidePidCtrl;
    public static PIDCoefficients lSlidePidConf = new PIDCoefficients(0.025, 0, 0);
    private PIDFController lSlidePidCtrl;
    public static PIDCoefficients rSlidePidConfVertical = new PIDCoefficients(0.008, 0, 0);
    private PIDFController rSlidePidCtrlVertical;
    public static PIDCoefficients lSlidePidConfVertical = new PIDCoefficients(0.008, 0, 0);
    private PIDFController lSlidePidCtrlVertical;
    public ServoPWMControl controlLeft = null;
    public ServoPWMControl controlRight = null;

    private HardwareMap hardwareMap;
    private Runnable updateRunnable;
//    private XCYBoolean slideZeroVelocity;

    private static RobotHardware instance = null;
    private boolean enabled;

    public ArmSubsystem armSubsystem;
    public SlideSubsystem slideSubsystem;
    public ClawSubsystem clawSubsystem;
    public GrabSubsystem grabSubsystem;
    public IntakeSubsystem intakeSubsystem;
    public WristSubsystem wristSubsystem;

    public int armOffset;

    public int armPosition;
    public double armPower;
    public int slidePosition;
    public double slidePower;

    public static RobotHardware getInstance() {
        if (instance == null) {
            instance = new RobotHardware();
        }
        instance.enabled = true;
        return instance;
    }

    //This MUST BE CALLED at the start of EVERY opMode.
    public void init(HardwareMap hardwareMap){

        this.hardwareMap = hardwareMap;
//        armPidCtrl = new PIDFController(armPidConf);
//        rSlidePidCtrl = new PIDFController(rSlidePidConf);
//        lSlidePidCtrl = new PIDFController(lSlidePidConf);
//        rSlidePidCtrlVertical = new PIDFController(rSlidePidConfVertical);
//        lSlidePidCtrlVertical = new PIDFController(lSlidePidConfVertical);
//
//        mArm = hardwareMap.get(DcMotorEx.class,"arm");
//
//        mSlideRight = hardwareMap.get(DcMotorEx.class,"slideRight");
//        mSlideLeft = hardwareMap.get(DcMotorEx.class,"slideLeft");
//        mSlideLeft.setDirection(DcMotorSimple.Direction.REVERSE);
////        mSlideRight.setDirection(DcMotorSimple.Direction.REVERSE);
//        mArm.setDirection(DcMotorSimple.Direction.REVERSE);
//
//
//        mIntakeLeft = hardwareMap.get(Servo.class,"intakeLeft");
//        mIntakeRight = hardwareMap.get(Servo.class,"intakeRight");
//        Wrist = hardwareMap.get(Servo.class,"wrist");
//        Grab = hardwareMap.get(Servo.class,"grab");
//
//        clawLeft = hardwareMap.get(Servo.class,"clawLeft");
//        clawRight = hardwareMap.get(Servo.class,"clawRight");
//
//        mTouchSensor = hardwareMap.get(TouchSensor.class,"touch");
////
//        mArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        mSlideRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        mSlideLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        Grab.setDirection(Servo.Direction.REVERSE);
//
//        mSlideRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        mSlideLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        mArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//
//        mSlideRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        mSlideLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//
//        mArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        controlLeft = new ServoPWMControl(mIntakeLeft);
//        controlRight = new ServoPWMControl(mIntakeRight);

        armSubsystem = new ArmSubsystem(hardwareMap, armOffset);
        slideSubsystem = new SlideSubsystem(hardwareMap);
        clawSubsystem = new ClawSubsystem(hardwareMap);
        grabSubsystem = new GrabSubsystem(hardwareMap);
        intakeSubsystem = new IntakeSubsystem(hardwareMap);
        wristSubsystem = new WristSubsystem(hardwareMap);


//        slideZeroVelocity = new XCYBoolean(()->mSlideLeft.getVelocity() == 0);

        this.armOffset = armOffset;
    }

    public void periodic(){
        //TODO: put all the subsystem periodics in here??
        slideSubsystem.periodic();
//        armSubsystem.periodic();

        slidePosition = slideSubsystem.getSlidesPosition();
        slidePower = slideSubsystem.getSlidePower();
        armPosition = armSubsystem.getArmPosition();
        armPower = armSubsystem.getArmPower();
    }

    //TODO: Stick the drivetrain & IMU stuff here as well


}
