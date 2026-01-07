package org.firstinspires.ftc.teamcode.Util;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.Pose;


//Make your poses using THIS website: https://visualizer.pedropathing.com

//All tutorials and explanations can be found HERE: https://pedropathing.com

@Configurable
public class Poses {


    public static Pose blueGoalTopStartFacing = new Pose(22, 124, Math.toRadians(137));
    public static Pose blueGoalTopStartOff = new Pose(32, 135.5, Math.toRadians(90));
    public static Pose blueShortScore = new Pose(48, 108);

    public static Pose obelisk = new Pose(72,148);

    public static Pose blueGoal = new Pose(16, 132);
    public static Pose redGoal = new Pose(128, 132);

    public static Pose bluePark = new Pose(105.5, 33);
    public static Pose blueParkCP = new Pose(105.5, 120);
    public static Pose redPark = new Pose(38.5, 33);
    public static Pose redParkCP = new Pose(38.5, 120);



}