package org.firstinspires.ftc.teamcode.Util;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.Pose;


//Make your poses using THIS website: https://visualizer.pedropathing.com

//All tutorials and explanations can be found HERE: https://pedropathing.com

@Configurable
public class Poses {



    public static Pose blueGoalTopStartFacing = new Pose(22, 124, Math.toRadians(144));
    public static Pose redGoalTopStartFacing = new Pose(122,127,Math.toRadians(37));
    public static Pose blueGoalTopStartOff = new Pose(33, 135.5, Math.toRadians(90));
    public static Pose blueShortScore = new Pose(48, 108, Math.toRadians(144));
    public static Pose redShortScore = new Pose(92, 100, Math.toRadians(37));

    public static Pose obelisk = new Pose(72,148);

    public static Pose blueGoal = new Pose(16, 132);
    public static Pose redGoal = new Pose(128, 132);

    public static Pose bluePark = new Pose(105.5, 33);
    public static Pose blueParkCP = new Pose(105.5, 120);
    public static Pose redPark = new Pose(38.5, 33);
    public static Pose redParkCP = new Pose(38.5, 120);


    public static Pose readyRedActiveTop = new Pose(92.000, 83.500);
    public static Pose readyBlueActiveTop = mirrorCoordinates(readyRedActiveTop, UniConstants.teamColor.BLUE);
    public static Pose redActiveTopStop = new Pose(112, 83.500);
    public static Pose blueActiveTopStop = mirrorCoordinates(redActiveTopStop, UniConstants.teamColor.BLUE);

    public static Pose readyRedActiveMid = new Pose(92.000, 60.000);
    public static Pose readyBlueActiveMid= mirrorCoordinates(readyRedActiveMid, UniConstants.teamColor.BLUE);
    public static Pose redActiveMidStop = new Pose(112, 60.000);
    public static Pose blueActiveMidStop = mirrorCoordinates(redActiveMidStop, UniConstants.teamColor.BLUE);

    public static Pose redMidCP = new Pose(87.14223693655666, 66.59257151401363);
    public static Pose blueMidCP = mirrorCoordinates(redMidCP, UniConstants.teamColor.BLUE);

    public static Pose blueParkAuto = new Pose(48, 128);
    public static Pose redParkAuto = mirrorCoordinates(blueParkAuto, UniConstants.teamColor.RED);


    public static Pose mirrorCoordinates(Pose pose, UniConstants.teamColor targetColor){
        double deltaX = Math.abs(72 - pose.getX());
        double angle = Math.toRadians((Math.toDegrees(pose.getHeading()) + 180) % 360);
        if(targetColor == UniConstants.teamColor.BLUE){
            return new Pose(72 - deltaX, pose.getY(), angle);
        } else {
            return new Pose(72 + deltaX, pose.getY(), angle);
        }
    }





}