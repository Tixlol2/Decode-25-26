package org.firstinspires.ftc.teamcode.Util;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.teamcode.Subsystems.RobotSubsystem;


//Make your poses using THIS website: https://visualizer.pedropathing.com

//All tutorials and explanations can be found HERE: https://pedropathing.com

@Configurable
public class Poses {
    public static Pose obelisk = new Pose(72, 148);

    public static Pose redGoal = new Pose(132, 136);
    public static Pose blueGoal = mirrorCoordinates(redGoal, RobotSubsystem.AllianceColor.BLUE);

    public static Pose blueClosePark = new Pose(24, 96);
    public static Pose redClosePark = mirrorCoordinates(blueClosePark, RobotSubsystem.AllianceColor.RED);

    public static Pose blueCloseAutoShoot = new Pose(56, 84);
    public static Pose redCloseAutoShoot = mirrorCoordinates(blueCloseAutoShoot, RobotSubsystem.AllianceColor.RED);

    public static Pose blueFarAutoShoot = new Pose(60, 18);
    public static Pose redFarAutoShoot = mirrorCoordinates(blueFarAutoShoot, RobotSubsystem.AllianceColor.RED);

    public static Pose blueCloseStart = new Pose(32.5, 135.5, Math.toRadians(90));

    public static Pose redCloseStart = Poses.mirrorCoordinates(blueCloseStart, RobotSubsystem.AllianceColor.RED);

    public static Pose farBlueIntakeControlPoint = new Pose(77.11392405063292, 38.65822784810126);
    public static Pose farRedIntakeControlPoint = mirrorCoordinates(farBlueIntakeControlPoint, RobotSubsystem.AllianceColor.RED);

    public static Pose farIntakeBlue = new Pose(12, 36);
    public static Pose farIntakeRed = mirrorCoordinates(farIntakeBlue, RobotSubsystem.AllianceColor.RED);

    public static Pose midBlueIntakeControlPoint = new Pose(79.13924050632912, 56.07594936708861);
    public static Pose midRedIntakeControlPoint = mirrorCoordinates(midBlueIntakeControlPoint, RobotSubsystem.AllianceColor.RED);

    public static Pose midIntakeBlue = new Pose(12, 60);
    public static Pose midIntakeRed = mirrorCoordinates(midIntakeBlue, RobotSubsystem.AllianceColor.RED);

    public static Pose closeBlueIntakeControlPoint = new Pose(78.32911392405065, 85.0379746835443);
    public static Pose closeRedIntakeControlPoint = mirrorCoordinates(closeBlueIntakeControlPoint, RobotSubsystem.AllianceColor.RED);

    public static Pose closeIntakeBlue = new Pose(16, 84);
    public static Pose closeIntakeRed = mirrorCoordinates(closeIntakeBlue, RobotSubsystem.AllianceColor.RED);

    public static Pose blueLeverBump = new Pose(16, 70);
    public static Pose blueLeverBumpControlPoint = new Pose(32.759493670886094, 70.45569620253166);
    public static Pose redLeverBump = mirrorCoordinates(blueLeverBump, RobotSubsystem.AllianceColor.RED);
    public static Pose redLeverBumpControlPoint = mirrorCoordinates(blueLeverBumpControlPoint, RobotSubsystem.AllianceColor.RED);



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