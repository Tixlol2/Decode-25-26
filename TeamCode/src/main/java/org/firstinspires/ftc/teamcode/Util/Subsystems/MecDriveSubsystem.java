package org.firstinspires.ftc.teamcode.Util.Subsystems;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.JoinedTelemetry;
import com.bylazar.telemetry.PanelsTelemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;

import org.firstinspires.ftc.teamcode.OpModes.NextFTCTeleop;
import org.firstinspires.ftc.teamcode.Util.Poses;
import org.firstinspires.ftc.teamcode.Util.UniConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.Constants;

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
    private static double changeInTurretAngle = 0;



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
        double x = 0, y = 0;

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

        // In Pedro Pathing: +Y is forward (90), +X is right (0)
        // atan2(y, x) measures angle from +X axis
        // We want angle where +Y is 0 (forward for the robot)
        // So: fieldAngle = atan2(y, x) gives us angle from +X
        // To get angle from +Y: subtract 90 (or add 270, same thing)
        double fieldAngleToTarget = Math.toDegrees(Math.atan2(y, x)) - 90;

        // Get robot heading in degrees (0 = facing +X, 90 = facing +Y)
        double robotHeading = getHeadingDegrees() - 90;

        // Calculate turret angle relative to robot
        changeInTurretAngle = fieldAngleToTarget - robotHeading;

        // Clamp to -180 to 180
        while (changeInTurretAngle > 180) changeInTurretAngle -= 360;
        while (changeInTurretAngle < -180) changeInTurretAngle += 360;

        //distance in meters
        distanceToGoal = Math.hypot(x, y) / 39.37;
    }

    public double getDistanceToGoal(){
        return distanceToGoal;
    }

    public double getCalculatedTurretAngle(){
        return changeInTurretAngle;
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
