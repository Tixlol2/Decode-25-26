package org.firstinspires.ftc.teamcode.Util.Subsystems;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.JoinedTelemetry;
import com.bylazar.telemetry.PanelsTelemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.paths.PathChain;

import org.firstinspires.ftc.teamcode.Util.Poses;

import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.ftc.ActiveOpMode;

@Configurable
public class MecDriveSubsystem implements Subsystem {
    public static boolean debug = false;
    public static MecDriveSubsystem INSTANCE = new MecDriveSubsystem();
    //For calculated turret angle
    private static double goalAngle = 0;
    private static double obeliskAngle = 0;
    private final boolean driving = false;
    //Class variables
    JoinedTelemetry telemetry;
    Robot.teamColor color = Robot.teamColor.BLUE;
    private Follower follower;
    private double distanceToGoal = 0;


    public MecDriveSubsystem() {
    }

    @Override
    public void initialize() {
        color = Robot.color;
        telemetry = new JoinedTelemetry(ActiveOpMode.telemetry(), PanelsTelemetry.INSTANCE.getFtcTelemetry());
        follower = PedroComponent.follower();

    }


    @Override
    public void periodic() {
        updateDistanceAndAngle();
    }

    public void startTele() {
        follower.startTeleopDrive();
        follower.update();
    }

    public void updateDistanceAndAngle() {
        double x = 0, y = 0, obX = 0, obY = 0;

        switch (Robot.color) {
            case BLUE:
                x = Poses.blueGoal.getX() - follower.getPose().getX();
                y = Poses.blueGoal.getY() - follower.getPose().getY();
                break;
            case RED:
                x = Poses.redGoal.getX() - follower.getPose().getX();
                y = Poses.redGoal.getY() - follower.getPose().getY();
                break;
        }

        obX = Poses.obelisk.getX() - follower.getPose().getX();
        obY = Poses.obelisk.getY() - follower.getPose().getY();

        // In Pedro Pathing: +Y is forward (90), +X is right (0)
        // atan2(y, x) measures angle from +X axis
        // We want angle where +Y is 0 (forward for the robot)
        // So: fieldAngle = atan2(y, x) gives us angle from +X
        // To get angle from +Y: subtract 90 (or add 270, same thing)
        double fieldAngleToTarget = Math.toDegrees(Math.atan2(y, x)) - 90;
        double targetObelisk = Math.toDegrees(Math.atan2(obY, obX)) - 90;

        // Get robot heading in degrees (0 = facing +X, 90 = facing +Y)
        double robotHeading = (getHeadingDegrees() - 90);

        // Calculate turret angle relative to robot
        goalAngle = fieldAngleToTarget - robotHeading;
        obeliskAngle = targetObelisk - robotHeading;

        // Clamp to -180 to 180
        while (goalAngle > 180) goalAngle -= 360;
        while (goalAngle < -180) goalAngle += 360;

        while (obeliskAngle > 180) obeliskAngle -= 360;
        while (obeliskAngle < -180) obeliskAngle += 360;

        //distance in meters
        distanceToGoal = Math.hypot(x, y) / 39.37;
    }

    public double getDistanceToGoal() {
        return distanceToGoal;
    }

    public double getGoalAngle() {
        return goalAngle;
    }

    public double getObeliskAngle() {
        return obeliskAngle;
    }

    public double getHeadingDegrees() {
        return Math.toDegrees(follower.getPose().getHeading());
    }

    public void setColor(Robot.teamColor color) {
        this.color = color;
    }

    public PathChain createParkPath() {
        return follower.pathBuilder()
                .addPath((follower.getPose().getX() >= 72) ?
                        new BezierCurve(follower.getPose(), color == Robot.teamColor.BLUE ? Poses.bluePark : Poses.redPark, color == Robot.teamColor.BLUE ? Poses.blueParkCP : Poses.redParkCP) :
                        new BezierLine(follower.getPose(), color == Robot.teamColor.BLUE ? Poses.bluePark : Poses.redPark))
                .setConstantHeadingInterpolation(Math.toRadians(90)).build();
    }

    public void sendTelemetry(Robot.loggingState state) {
        switch (state) {
            case DISABLED:
                break;
            case ENABLED:
                telemetry.addLine("START OF MEC DRIVE LOG");
                telemetry.addData("Pose X ", follower.getPose().getX());
                telemetry.addData("Pose Y ", follower.getPose().getY());
                telemetry.addData("Pose Heading Degrees ", Math.toDegrees(follower.getPose().getHeading()));
                telemetry.addData("Distance to Goal: ", distanceToGoal);
                telemetry.addData("Color: ", color);
                telemetry.addLine("END OF MEC DRIVE LOG");
                break;
            case EXTREME:
                telemetry.addLine("START OF MEC DRIVE LOG");
                telemetry.addData("Follower Debug ", follower.debug());
                telemetry.addLine("END OF MEC DRIVE LOG");
                break;
        }
    }

    public void setTelemetry(JoinedTelemetry telemetry) {
        this.telemetry = telemetry;
    }


}
