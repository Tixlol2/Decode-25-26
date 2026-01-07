package org.firstinspires.ftc.teamcode.Util.Subsystems;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.JoinedTelemetry;
import com.bylazar.telemetry.PanelsTelemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

import org.firstinspires.ftc.teamcode.OpModes.NextFTCTeleop;
import org.firstinspires.ftc.teamcode.Util.Poses;
import org.firstinspires.ftc.teamcode.Util.Timer;
import org.firstinspires.ftc.teamcode.Util.UniConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.Constants;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.utility.LambdaCommand;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.ftc.ActiveOpMode;

@Configurable
//TODO: Tune all constants to ensure correctness awesome - prob not needed
public class MecDriveSubsystem implements Subsystem {
    //Class variables
    JoinedTelemetry telemetry;
    UniConstants.teamColor color = NextFTCTeleop.color;
    public static boolean debug = false;
    private Follower follower;
    private double distanceToGoal = 0;

    public static final MecDriveSubsystem INSTANCE = new MecDriveSubsystem();

    //For calculated turret angle
    private static double goalAngle = 0;
    private static double obeliskAngle = 0;



    public MecDriveSubsystem(){}

    @Override
    public void initialize(){
        telemetry = new JoinedTelemetry(ActiveOpMode.telemetry(), PanelsTelemetry.INSTANCE.getFtcTelemetry());
        follower = Constants.createFollower(ActiveOpMode.hardwareMap());
        follower.setPose(new Pose());
        follower.update();
    }


    @Override
    public void periodic(){
        color = NextFTCTeleop.color;
        updateDistanceAndAngle();
        follower.update();
    }

    public void startTele(){
        follower.startTeleopDrive();
        follower.update();
    }
    public void updateTeleop(double forward, double strafe, double rotation, boolean botCentric){
        follower.setTeleOpDrive(forward, strafe, rotation, botCentric);
        follower.update();
    }

    public void updateDistanceAndAngle() {
        double x = 0, y = 0, obX = 0, obY = 0;

        switch (NextFTCTeleop.color) {
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
        double robotHeading = getHeadingDegrees() - 90;

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

    public Command FollowPath(PathChain path){
        return new LambdaCommand("Follow Path")
                .setStart(() -> MecDriveSubsystem.INSTANCE.getFollower().followPath(path))
                .setIsDone(() -> !follower.isBusy())
                .setInterruptible(false)
                .requires(this);
    }

    public Command FollowPath(PathChain path, boolean holdEnd){
        return new LambdaCommand("Follow Path Hold End: " + holdEnd)
                .setStart(() -> MecDriveSubsystem.INSTANCE.getFollower().followPath(path, holdEnd))
                .setIsDone(() -> follower.getPose().roughlyEquals(path.endPose(), 3))
                .setInterruptible(false)
                .requires(MecDriveSubsystem.INSTANCE);
    }

    public Command FollowPath(PathChain path, boolean holdEnd, double maxPower){
        return new LambdaCommand("Follow Path Hold End: " + holdEnd)
                .setStart(() -> MecDriveSubsystem.INSTANCE.getFollower().followPath(path, maxPower, holdEnd))
                .setIsDone(() -> follower.getPose().roughlyEquals(path.endPose(), 3))
                .setInterruptible(false)
                .requires(MecDriveSubsystem.INSTANCE);
    }

    public Command FollowPathTime(PathChain path, boolean holdEnd, double seconds){
        Timer timer = new Timer();
        return new LambdaCommand("Follow Path Time Hold End: " + holdEnd)
                .setStart(() -> {MecDriveSubsystem.INSTANCE.getFollower().followPath(path, holdEnd); timer.reset();})
                .setIsDone(() -> timer.getTimeSeconds() > seconds)
                .setInterruptible(false)
                .requires(MecDriveSubsystem.INSTANCE);
    }

    public Command FollowPathTime(PathChain path, boolean holdEnd, double maxPower, double seconds){
        Timer timer = new Timer();
        return new LambdaCommand("Follow Path Time Hold End: " + holdEnd)
                .setStart(() -> {MecDriveSubsystem.INSTANCE.getFollower().followPath(path, maxPower, holdEnd); timer.reset();})
                .setIsDone(() -> timer.getTimeSeconds() > seconds)
                .setInterruptible(false)
                .requires(MecDriveSubsystem.INSTANCE);
    }



    public double getDistanceToGoal(){
        return distanceToGoal;
    }

    public double getCalculatedTurretAngle(){
        return goalAngle;
    }

    public double getObeliskAngle(){
        return obeliskAngle;
    }


    public void setPose(Pose pose) {
        follower.setPose(pose);
    }

    public double getHeadingDegrees(){
        return Math.toDegrees(follower.getPose().getHeading());
    }

    public void setColor(UniConstants.teamColor col){color = col;}


    public Follower getFollower(){
        return follower;
    }

    public void sendTelemetry(UniConstants.loggingState state){
        switch(state){
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

}
