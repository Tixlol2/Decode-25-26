package org.firstinspires.ftc.teamcode.Util;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.teamcode.Subsystems.RobotSubsystem;


//Make your poses using THIS website: https://visualizer.pedropathing.com

//All tutorials and explanations can be found HERE: https://pedropathing.com

@Configurable
public class Poses {
    public static Pose obelisk = new Pose(72, 148);

    public static Pose redGoal = new Pose(128, 132);
    public static Pose blueGoal = mirrorCoordinates(redGoal, RobotSubsystem.AllianceColor.BLUE);




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