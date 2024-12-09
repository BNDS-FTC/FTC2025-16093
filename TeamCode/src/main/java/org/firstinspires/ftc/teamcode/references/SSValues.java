package org.firstinspires.ftc.teamcode.references;

import com.acmerobotics.dashboard.config.Config;

@Config
public class SSValues {
    public static final int AUTO_ARM_OFFSET = 200;

    //****DO NOT SET ARM VALUES LARGER THAN 1300****
    public static final int ARM_DEFAULT = 0;
    public static final int ARM_UP = 1110;
    public static final int ARM_HANG1 = 680;

    public static final int SLIDE_MIN = 0;
    public static final int SLIDE_SLIGHTLY_LONGER = 293;
    public static final int SLIDE_INTAKE_NEAR = 147;
    public static final int SLIDE_AUTO_INTAKE_LAST= 754;
    public static final int SLIDE_AUTO_INTAKE_YELLOW = 440;
    public static final int SLIDE_INTAKE_FAR = 513;
    public static final int SLIDE_HIGH_CHAMBER_AIM = 969;
    public static final int SLIDE_HIGH_CHAMBER_PLACE = 691;
    public static final int SLIDE_ASCENT_UP = 1321;
    public static final int SLIDE_ASCENT_DOWN = 734;
    public static final int SLIDE_MAX = 1556;

    public static final double WRIST_DEFAULT = 0.99;
    public static final double WRIST_RELEASE = 0.75;
    public static final double WRIST_ABOVE_SAMPLES = 0.25;

    public static final double WRIST_INTAKE = 0.07;
    public static final double WRIST_HIGH_CHAMBER = 0.3;

    public static final double GRAB_DEFAULT = 0.64;
    public static final double GRAB_OPEN = 0.8;
    public static final double GRAB_CLOSED = 0.49;

    public static final double CLAW_LEFT_OPEN = 0;
    public static final double CLAW_LEFT_CLOSE = 0.35;
    public static final double CLAW_RIGHT_OPEN = 1;
    public static final double CLAW_RIGHT_CLOSE = 0.55;


    public static final double CONTINUOUS_SPIN = 0;
    public static final double CONTINUOUS_STOP = 0.22;
    public static final double CONTINUOUS_STOP_OPPOSITE=0.77;
    public static final double CONTINUOUS_SPIN_OPPOSITE = 1;
}