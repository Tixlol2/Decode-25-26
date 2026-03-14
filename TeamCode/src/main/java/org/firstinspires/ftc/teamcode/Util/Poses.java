package org.firstinspires.ftc.teamcode.Util;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.teamcode.Subsystems.RobotSubsystem;


//Make your poses using THIS website: https://visualizer.pedropathing.com

//All tutorials and explanations can be found HERE: https://pedropathing.com

@Configurable
public class Poses {
    public static Pose obelisk = new Pose(72, 148);

    public static Pose redGoal = new Pose(126, 146);
    public static Pose blueGoal = mirrorCoordinates(redGoal, RobotSubsystem.AllianceColor.BLUE);

    public static Pose blueClosePark = new Pose(56, 132);
    public static Pose redClosePark = mirrorCoordinates(blueClosePark, RobotSubsystem.AllianceColor.RED);

    public static Pose blueCloseAutoShoot = new Pose(54, 94);
    public static Pose redCloseAutoShoot = mirrorCoordinates(blueCloseAutoShoot, RobotSubsystem.AllianceColor.RED);

    public static Pose blueCloseStart = new Pose(32.5, 135.5, Math.toRadians(90));
    public static Pose redCloseStart = Poses.mirrorCoordinates(blueCloseStart, RobotSubsystem.AllianceColor.RED).setHeading(Math.toRadians(90));

    public static Pose farBlueIntakeControlPoint = new Pose(77.11392405063292, 30);
    public static Pose farRedIntakeControlPoint = mirrorCoordinates(farBlueIntakeControlPoint, RobotSubsystem.AllianceColor.RED);

    public static Pose farIntakeBlue = new Pose(10, 30);
    public static Pose farIntakeRed = mirrorCoordinates(farIntakeBlue, RobotSubsystem.AllianceColor.RED);

    public static Pose midBlueIntakeControlPoint = new Pose(72, 64);
    public static Pose midRedIntakeControlPoint = mirrorCoordinates(midBlueIntakeControlPoint, RobotSubsystem.AllianceColor.RED);

    public static Pose midIntakeBlue = new Pose(24, 51);
    public static Pose midIntakeRed = mirrorCoordinates(midIntakeBlue, RobotSubsystem.AllianceColor.RED);

    public static Pose closeBlueIntakeControlPoint = new Pose(60, 85.44303797468353);
    public static Pose closeRedIntakeControlPoint = mirrorCoordinates(closeBlueIntakeControlPoint, RobotSubsystem.AllianceColor.RED);

    public static Pose closeIntakeBlue = new Pose(26, 73);
    public static Pose closeIntakeRed = mirrorCoordinates(new Pose(24, 76), RobotSubsystem.AllianceColor.RED);


    public static Pose blueFarStart = new Pose(56, 8.5, Math.toRadians(90));
    public static Pose redFarStart = Poses.mirrorCoordinates(blueFarStart, RobotSubsystem.AllianceColor.RED).setHeading(Math.toRadians(90));

    public static Pose blueLever = new Pose(16, 77);
    public static Pose redLever = mirrorCoordinates(blueLever, RobotSubsystem.AllianceColor.RED);

    public static Pose blueFarShoot = new Pose(69, 22);
    public static Pose redFarShoot = mirrorCoordinates(new Pose(83, 22), RobotSubsystem.AllianceColor.RED);

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