package org.firstinspires.ftc.teamcode.Util;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.teamcode.Subsystems.RobotSubsystem;


//Make your poses using THIS website: https://visualizer.pedropathing.com

//All tutorials and explanations can be found HERE: https://pedropathing.com

@Configurable
public class Poses {


    public static Pose blueGoalTopStartFacing = new Pose(22, 124, Math.toRadians(144));
    public static Pose redGoalTopStartFacing = new Pose(122, 127, Math.toRadians(37));
    public static Pose blueShortScore = new Pose(44, 112);
    public static Pose redShortScore = new Pose(96, 108);
    public static Pose blueFarScore = new Pose(44, 112);
    public static Pose redFarScore = new Pose(96, 108);

    public static Pose obelisk = new Pose(72, 148);

    public static Pose redGoal = new Pose(128, 132);
    public static Pose blueGoal = mirrorCoordinates(redGoal, RobotSubsystem.AllianceColor.BLUE);

    public static Pose bluePark = new Pose(105.5, 33);
    public static Pose blueParkCP = new Pose(105.5, 120);
    public static Pose redPark = new Pose(38.5, 33);
    public static Pose redParkCP = new Pose(38.5, 120);


    public static Pose redActiveTopStop = new Pose(120, 90);
    public static Pose blueActiveTopStop = mirrorCoordinates(redActiveTopStop, RobotSubsystem.AllianceColor.BLUE);

    public static Pose redActiveMidStop = new Pose(122, 63);
    public static Pose blueActiveMidStop = mirrorCoordinates(redActiveMidStop, RobotSubsystem.AllianceColor.BLUE);

    public static Pose redMidCP = new Pose(87.14223693655666, 66.59257151401363);
    public static Pose blueMidCP = mirrorCoordinates(redMidCP, RobotSubsystem.AllianceColor.BLUE);

    public static Pose blueShortParkAuto = new Pose(48, 128);
    public static Pose redShortParkAuto = mirrorCoordinates(blueShortParkAuto, RobotSubsystem.AllianceColor.RED);
    public static Pose blueFarParkAuto = new Pose(24, 12);
    public static Pose redFarParkAuto = mirrorCoordinates(blueFarParkAuto, RobotSubsystem.AllianceColor.RED);


    public static Pose mirrorCoordinates(Pose pose, RobotSubsystem.AllianceColor targetColor) {
        double deltaX = Math.abs(72 - pose.getX());
        double angle = Math.toRadians((Math.toDegrees(pose.getHeading()) + 180) % 360);
        if (targetColor == RobotSubsystem.AllianceColor.BLUE) {
            return new Pose(72 - deltaX, pose.getY(), angle);
        } else {
            return new Pose(72 + deltaX, pose.getY(), angle);
        }
    }


}