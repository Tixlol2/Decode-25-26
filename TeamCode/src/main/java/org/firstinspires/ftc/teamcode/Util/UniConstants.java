package org.firstinspires.ftc.teamcode.Util;


import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import java.util.ArrayList;
import java.util.List;

@Configurable
public class UniConstants {

/*
CONFIG

//TODO: Update all of this ASAP

//Control Hub
    //CHM0 = ACTIVE
    //CHM1 = ROTA
    //CHM2 = FLM
    //CHM3 = BLM

    //CHI2C0 = imu
    //CHI2C1 = CSF
    //CHI2C2 = CSL
    //CHI2C3 = pp

//Expansion Hub
    //EHM0 = LAUNCHER
    //EHM1 =
    //EHM2 = FRM
    //EHM3 = BRM

    //EHI2C0 = CSR

    //EHS0 = FBS
    //EHS5 = FLS

*/

    //Drive
    public static final String DRIVE_FRONT_LEFT_STRING = "FLM";
    public static final String DRIVE_FRONT_RIGHT_STRING = "FRM";
    public static final String DRIVE_BACK_LEFT_STRING = "BLM";
    public static final String DRIVE_BACK_RIGHT_STRING = "BRM";
    public static final DcMotorEx.Direction DRIVE_FRONT_LEFT_DIRECTION = DcMotorEx.Direction.REVERSE;
    public static final DcMotorEx.Direction DRIVE_FRONT_RIGHT_DIRECTION = DcMotorEx.Direction.REVERSE;
    public static final DcMotorEx.Direction DRIVE_BACK_LEFT_DIRECTION = DcMotorEx.Direction.FORWARD;
    public static final DcMotorEx.Direction DRIVE_BACK_RIGHT_DIRECTION = DcMotorEx.Direction.REVERSE;
    public static final String PINPOINT_STRING = "pp";
    //Outtake Subsystem
    public static final String LAUNCHER_TOP_STRING = "LAUNCHTOP";
    public static final String LAUNCHER_BOTTOM_STRING = "LAUNCHBOT";
    //Rotary Subsystem
    public static final String ACTIVE_INTAKE_STRING = "ACTIVE";
    //Flickers
    public static final String FLICKER_BACK_STRING = "FBS";
    public static final String FLICKER_RIGHT_STRING = "FRS";
    public static final String FLICKER_LEFT_STRING = "FLS";
    //Turret Control
    public static final String TURRET_STRING = "TURR";
    public static final double MOTOR_TURRET_RATIO = (double) 28 / 92;
    public static final double TURRET_TICKS_PER_DEGREE = (537.7 / MOTOR_TURRET_RATIO) / 360;
    //Color sensors
    public static final String COLOR_SENSOR_SLOT_BACK_STRING = "CSB";
    public static final String COLOR_SENSOR_SLOT_RIGHT_STRING = "CSR";
    public static final String COLOR_SENSOR_SLOT_LEFT_STRING = "CSL";
    //Light Names
    public static final String LIGHT_BACK_STRING = "BACK";
    public static final String LIGHT_RIGHT_STRING = "RIGHT";
    public static final String LIGHT_LEFT_STRING = "LEFT";
    public static final ArrayList<Integer> obeliskIDs = new ArrayList<>(List.of(21, 22, 23));
    //Flickers
    public static final double FLICKER_BACK_UP = .93;
    public static final double FLICKER_BACK_DOWN = .55;
    public static final double FLICKER_RIGHT_DOWN = .52;
    public static final double FLICKER_RIGHT_UP = .15;
    public static final double FLICKER_LEFT_DOWN = .22;
    public static final double FLICKER_LEFT_UP = .62;
    public static double FAST_FLICKER_TIME_UP = .75;
    public static double FAST_FLICKER_TIME_DOWN = .125;
    //Hood
    public static final String HOOD_STRING = "HOOD";



}
