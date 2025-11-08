package org.firstinspires.ftc.teamcode.Util.Subsystems;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.JoinedTelemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Util.Poses;
import org.firstinspires.ftc.teamcode.Util.UniConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.Constants;

import dev.nextftc.core.subsystems.Subsystem;

@Configurable
//TODO: Tune all constants to ensure correctness awesome
public class MecDriveSubsystem implements Subsystem {
    //Class variables
    JoinedTelemetry telemetry;
    UniConstants.teamColor color;
    public static boolean debug = false;
    private Follower follower;

    //For ensuring that PP is reset
    public static GoBildaPinpointDriver pinpoint;

    //For calculated turret angle
    private static double changeInTurretAngle = 0;


    public MecDriveSubsystem(HardwareMap hardwareMap, JoinedTelemetry telemetry, UniConstants.teamColor color){
        this.color = color;
        this.telemetry = telemetry;
        follower = Constants.createFollower(hardwareMap);
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, UniConstants.PINPOINT_STRING);
        follower.setPose(new Pose());
        follower.update();

    }




    @Override
    public void periodic(){
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

    public double updateDistanceAndAngle(UniConstants.teamColor color) {
        //Returns distance between goal in meters, it also updates the turretTargetAngle
        double x = 0,y = 1;
        switch (color){
            case BLUE:
                x = follower.getPose().getX() - Poses.blueGoal.getX();
                y = follower.getPose().getY() - Poses.blueGoal.getY();
                break;
            case RED:
                x = follower.getPose().getX() - Poses.redGoal.getX();
                y = follower.getPose().getY() - Poses.redGoal.getY();
        }

        changeInTurretAngle = (Math.toDegrees(Math.atan2(x,y))) + Math.toDegrees(follower.getPose().getHeading());
        return Math.hypot(x,y) / 39.37;

    }

    public double getCalculatedTurretAngle(){
        return color == UniConstants.teamColor.RED ? -changeInTurretAngle : changeInTurretAngle;
    }

    public void resetPinpoint(){
        pinpoint.resetPosAndIMU();
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
