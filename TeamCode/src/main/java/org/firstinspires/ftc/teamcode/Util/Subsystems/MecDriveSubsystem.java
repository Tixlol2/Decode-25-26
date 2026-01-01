package org.firstinspires.ftc.teamcode.Util.Subsystems;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.JoinedTelemetry;
import com.bylazar.telemetry.PanelsTelemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;

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

    public double updateDistanceAndAngle() {
        //Returns distance between goal in meters, it also updates the turretTargetAngle
        double x = 0,y = 1;
        switch (NextFTCTeleop.color){
            case BLUE:
                x = Poses.blueGoal.getX() - follower.getPose().getX();
                y = Poses.blueGoal.getY() - follower.getPose().getY();
                break;
            case RED:
                x =  Poses.redGoal.getX() - follower.getPose().getX();
                y =  Poses.redGoal.getY() - follower.getPose().getY();
                break;
        }

        changeInTurretAngle = 180 + (-(Math.toDegrees(Math.atan2(y,x))) - Math.toDegrees(follower.getPose().getHeading()));
        return Math.hypot(x,y) / 39.37;

    }

    public double getCalculatedTurretAngle(){
        return NextFTCTeleop.color == UniConstants.teamColor.RED ? -changeInTurretAngle : changeInTurretAngle;
    }


    public void setPose(Pose pose) {
        follower.setPose(pose);
    }

    public double getHeadingDegrees(){
        return Math.toDegrees(follower.getPose().getHeading());
    }

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
                telemetry.addData("Calculated Angle ", getCalculatedTurretAngle());
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
