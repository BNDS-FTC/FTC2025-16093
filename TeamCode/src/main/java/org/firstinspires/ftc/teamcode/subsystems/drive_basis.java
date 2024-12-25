package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.*;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.sun.tools.javac.tree.DCTree;

import org.firstinspires.ftc.teamcode.gobildapinpoint.GoBildaPinpointDriver;

public class drive_basis extends SubsystemBase {

//    private final MecanumDrive drive;
    private boolean FIELD_CENTRIC = false; // Toggle for field-centric control
    private GoBildaPinpointDriver odo;
    private DcMotorEx leftFront, leftRear, rightRear, rightFront;


    /**
     * Creates a new MecanumDrive subsystem.
     * @param fieldCentric Control mode
     */
    public drive_basis(boolean fieldCentric) {
        // Initialize the MecanumDrive using FTCLib's Motor class
//        drive = new MecanumDrive(
//                new Motor(hardwareMap, "leftFront"),
//                new Motor(hardwareMap, "rightFront"),
//                new Motor(hardwareMap, "leftBack"),
//                new Motor(hardwareMap, "rightBack")
//        );
        this.FIELD_CENTRIC=fieldCentric;
        odo = hardwareMap.get(GoBildaPinpointDriver.class,"odo");
        odo.setOffsets(-100,-115);
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD);
    }

    @Override
    public void periodic(){
        if(FIELD_CENTRIC){
            drive(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x, odo.getHeading(),1,1);
        }else{
            drive(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x,1,1);
        }
    }
    /**
     * Drives the robot using field-centric control.
     *
     * @param leftX   Strafe input.
     * @param leftY   Forward/backward input.
     * @param rightX  Rotation input.     
     * @param heading The current heading of the robot (used for field-centric).
     */
    public void drive(double leftX, double leftY, double rightX, double heading, double driveCoefficientTrans, double driveCoefficientRot) {
//        drive.driveFieldCentric(leftX,leftY,rightX,heading,false);

        double rotX = leftX * Math.cos(-heading) - leftY * Math.sin(-heading);
        double rotY = leftX * Math.sin(-heading) + leftY * Math.cos(-heading);

        rotX = rotX * 1.1;

        double y = leftY*-driveCoefficientTrans;
        double x = leftX*driveCoefficientTrans;
        double rx = rightX*-driveCoefficientRot;

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


    /**
     * Drives the robot using robotic-centric control.
     * @param leftX   Strafe input.
     * @param leftY   Forward/backward input.
     * @param rightX  Rotation input.
     */
    public void drive(double leftX, double leftY, double rightX,double driveCoefficientTrans,double driveCoefficientRot) {
//        drive.driveRobotCentric(leftX,leftY,rightX,false);
        double y = leftY*-driveCoefficientTrans;
        double x = leftX*driveCoefficientTrans;
        double rx = rightX*-driveCoefficientRot;

        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        double frontLeftPower = (y + x + rx) / denominator;
        double backLeftPower = (y - x + rx) / denominator;
        double frontRightPower = (y - x - rx) / denominator;
        double backRightPower = (y + x - rx) / denominator;

        leftFront.setPower(frontLeftPower);
        leftRear.setPower(backLeftPower);
        rightFront.setPower(frontRightPower);
        rightRear.setPower(backRightPower);
    }
}