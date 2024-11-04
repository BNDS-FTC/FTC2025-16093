package org.firstinspires.ftc.teamcode.references;

import com.acmerobotics.dashboard.config.Config;

@Config
public class SSValues {
    //****DO NOT SET ARM VALUES LARGER THAN 1750****

    public static final int ARM_DEFAULT = 0;
//    public static final int ARM_INTAKE_NEAR = 150;
//    public static final int ARM_INTAKE_FAR = 310;
    public static final int ARM_LOW_BASKET = 1400;
    public static final int ARM_HIGH_BASKET = 1400;
    public static final int ARM_HANG1 = 680;
    public static final int ARM_HIGH_CHAMBER_AIM = 760;
    public static final int ARM_HIGH_CHAMBER_PLACE = 640;
    public static final int ARM_GET_FROM_HP = 500;

    public static int[][] customPos = {{150,400},{230,600},{250,800},{270,1000},{300,1200},{305,1300}};

    public static final int SLIDE_MIN = 0;
    public static final int SLIDE_INTAKE_NEAR = 400;
    public static final int SLIDE_INTAKE_2 = 800;
    public static final int SLIDE_INTAKE_3 = 1200;
    public static final int SLIDE_MAX = 1400;

    public static final double WRIST_DEFAULT = 0.99;
    public static final double WRIST_RELEASE = 0.99;
    public static final double WRIST_INTAKE = 0;
    public static final double WRIST_HIGH_CHAMBER = 0.3;

    public static final double GRAB_DEFAULT = 0.65;
    public static final double GRAB_OPEN = 0.8;
    public static final double GRAB_CLOSED = 0.51;

    public static final double CONTINUOUS_SPIN = 0;
    public static final double CONTINUOUS_STOP = 0.28;
    public static final double CONTINUOUS_STOP_OPPOSITE=0.72;
    public static final double CONTINUOUS_SPIN_OPPOSITE = 1;
}
