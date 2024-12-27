package org.firstinspires.ftc.teamcode.references;

import com.acmerobotics.dashboard.config.Config;

@Config
public class SSValues {
    public static final int AUTO_ARM_OFFSET = 400;

    //****DO NOT SET ARM VALUES LARGER THAN ARM_UP****
    public static final int ARM_DOWN = 0;
    public static final int ARM_UP = 1120;
    public static final int ARM_HANG1 = 558;
    public static final int ARM_SLIGHTLY_HIGHER = 200;


    public static int maxValue = 1690;//Adjust this variable to assume that everything else changes in proportion.
    public static final int SLIDE_MIN = 0;
    public static final int SLIDE_SLIGHTLY_LONGER = (int)(maxValue*0.1903);
    public static final int SLIDE_LONGER = (int)(maxValue*0.3);
    public static final int SLIDE_INTAKE_NEAR = (int)(maxValue*0.0955);
    public static final int SLIDE_AUTO_INTAKE_LAST_BLUE = (int)(maxValue*0.54);
    public static final int SLIDE_AUTO_INTAKE_LAST_RED = (int)(maxValue*0.50);
    public static final int SLIDE_AUTO_INTAKE_FIRST= (int)(maxValue*0.5538);
    public static final int SLIDE_AUTO_INTAKE_YELLOW = (int)(maxValue*0.325);
    public static final int SLIDE_INTAKE_FAR = (int)(maxValue*0.58);
    public static final int SLIDE_HIGH_CHAMBER_AIM_AUTO = (int)(maxValue*0.43);
    public static final int SLIDE_HIGH_CHAMBER_AIM_TELEOP = (int)(maxValue*0.64);
    public static final int SLIDE_HIGH_CHAMBER_PLACE = (int)(maxValue*0.4487);
    public static final int SLIDE_ASCENT_UP = (int)(maxValue*0.8578);
    public static final int SLIDE_ASCENT_DOWN = 50;
    public static final int SLIDE_MAX = maxValue;

    public static final double WRIST_DEFAULT = 0.95;
    public static final double WRIST_RELEASE = 0.75;
    public static final double WRIST_ABOVE_SAMPLES = 0.25;

    public static final double WRIST_INTAKE = -0.05;
    public static final double WRIST_HIGH_CHAMBER = 0.3;

    public static final double GRAB_DEFAULT = 0.64;
    public static final double GRAB_OPEN = 0.8;
    public static final double GRAB_CLOSED = 0.5;
    public static final double AUTO_GRAB_CLOSED = 0.45;

    public static final double CLAW_LEFT_OPEN = 0;
    public static final double CLAW_LEFT_CLOSE = 0.35;
    public static final double CLAW_RIGHT_OPEN = 0.9;
    public static final double CLAW_RIGHT_CLOSE = 0.55;


    public static final double CONTINUOUS_SPIN = 0.67;
    public static final double CONTINUOUS_STOP = 0.5;
    public static final double CONTINUOUS_STOP_OPPOSITE=0.77;
    public static final double CONTINUOUS_SPIN_OPPOSITE = 0.33;

    public static final double SLIDE_LOCK_DEFAULT = 0;
    public static final double SLIDE_LOCK_LOCKED_TIGHT = 0.14;
    public static final double SLIDE_LOCK_LOCKED_NORMAL = 0.13;
}