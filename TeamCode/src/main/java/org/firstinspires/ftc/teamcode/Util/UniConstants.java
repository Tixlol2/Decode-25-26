package org.firstinspires.ftc.teamcode.Util;


import android.graphics.Color;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.vision.opencv.ColorBlobLocatorProcessor;
import org.firstinspires.ftc.vision.opencv.ColorRange;
import org.firstinspires.ftc.vision.opencv.ImageRegion;

@Configurable
public class UniConstants {

/*
CONFIG

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
    //EHM0 = FR
    //EHM1 = BR
    //EHM2 = TURR
    //EHM3 = LAUNCH

    //EHI2C0 = CSR

    //EHS5 = BS

*/

    //Drive
    public static final String DRIVE_FRONT_LEFT_STRING = "FLM";
    public static final String DRIVE_FRONT_RIGHT_STRING = "FRM";
    public static final String DRIVE_BACK_LEFT_STRING = "BLM";
    public static final String DRIVE_BACK_RIGHT_STRING = "BRM";
    public static final DcMotorEx.Direction DRIVE_FRONT_LEFT_DIRECTION = DcMotorEx.Direction.FORWARD;
    public static final DcMotorEx.Direction DRIVE_FRONT_RIGHT_DIRECTION = DcMotorEx.Direction.FORWARD;
    public static final DcMotorEx.Direction DRIVE_BACK_LEFT_DIRECTION = DcMotorEx.Direction.REVERSE;
    public static final DcMotorEx.Direction DRIVE_BACK_RIGHT_DIRECTION = DcMotorEx.Direction.FORWARD;
    public static final String PINPOINT_STRING = "pp";

    //Enums
    public enum loggingState{
        DISABLED,
        ENABLED,
        EXTREME
    }

    public enum slotState{
        PURPLE,
        GREEN,
        EMPTY,
        BETWEEN
    }

    public enum teamColor {
        RED,
        BLUE
    }



    //Outtake Subsystem
    public static final String LAUNCHER_STRING  = "LAUNCHER";
    public static final String TURRET_ROTATION_STRING = "TURR";

    //Rotary Subsystem
    public static final String ACTIVE_INTAKE_STRING = "ACTIVE";
    public static final DcMotorSimple.Direction ACTIVE_DIRECTION = DcMotorSimple.Direction.REVERSE;
    public static final String ROTARY_STRING = "ROTARY";
    public static final DcMotorSimple.Direction ROTARY_DIRECTION = DcMotorSimple.Direction.REVERSE;
    public static final int SPACE_BETWEEN_ROTARY_SLOTS = 300;
    public static final String BALL_SERVO_STRING = "BS";
    public static final double SERVO_PASS = 0;
    public static final double SERVO_TRANSFER = 1;
    //Color sensors
    public static final String COLOR_SENSOR_SLOT_FRONT_STRING = "CSF";
    public static final String COLOR_SENSOR_SLOT_RIGHT_STRING = "CSR";
    public static final String COLOR_SENSOR_SLOT_LEFT_STRING = "CSL";

    //Launcher Speed Calculation variables
    public static final double ANGLE_OF_LAUNCHER_IN_DEGREES = 35;
    public static  final double HEIGHT_OF_ROBOT_IN_METERS = 0.35; //TODO: Check to make sure this is right
    public static  final double HEIGHT_TO_GOAL_WITH_CLEARANCE_METERS = (1.11125) - (HEIGHT_OF_ROBOT_IN_METERS);
    public static  final double MOTOR_TO_TURRET_RATIO = (double) 24 /155; //Motor to Turret
    public static  final double TURRET_TICKS_PER_DEGREE = 537.7/360;



    //Artifact locator processors
    public static final ColorBlobLocatorProcessor colorLocatorGreen = new ColorBlobLocatorProcessor.Builder()
            .setTargetColorRange(ColorRange.ARTIFACT_GREEN)   // Use a predefined color match
            .setContourMode(ColorBlobLocatorProcessor.ContourMode.EXTERNAL_ONLY)
            .setRoi(ImageRegion.asUnityCenterCoordinates(-0.75, 0.75, 0.75, -0.75))
            .setDrawContours(true)   // Show contours on the Stream Preview
            .setBoxFitColor(0)       // Disable the drawing of rectangles
            .setCircleFitColor(Color.rgb(255, 255, 0)) // Draw a circle
            .setBlurSize(5)          // Smooth the transitions between different colors in image

            // the following options have been added to fill in perimeter holes.
            .setDilateSize(15)       // Expand blobs to fill any divots on the edges
            .setErodeSize(15)        // Shrink blobs back to original size
            .setMorphOperationType(ColorBlobLocatorProcessor.MorphOperationType.CLOSING)

            .build();

    public static final ColorBlobLocatorProcessor colorLocatorPurple = new ColorBlobLocatorProcessor.Builder()
            .setTargetColorRange(ColorRange.ARTIFACT_PURPLE)   // Use a predefined color match
            .setContourMode(ColorBlobLocatorProcessor.ContourMode.EXTERNAL_ONLY)
            .setRoi(ImageRegion.asUnityCenterCoordinates(-0.75, 0.75, 0.75, -0.75))
            .setDrawContours(true)   // Show contours on the Stream Preview
            .setBoxFitColor(0)       // Disable the drawing of rectangles
            .setCircleFitColor(Color.rgb(255, 255, 0)) // Draw a circle
            .setBlurSize(5)          // Smooth the transitions between different colors in image

            // the following options have been added to fill in perimeter holes.
            .setDilateSize(15)       // Expand blobs to fill any divots on the edges
            .setErodeSize(15)        // Shrink blobs back to original size
            .setMorphOperationType(ColorBlobLocatorProcessor.MorphOperationType.CLOSING)

            .build();



}
