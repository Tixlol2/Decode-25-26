package org.firstinspires.ftc.teamcode.Util.Subsystems;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.JoinedTelemetry;
import com.bylazar.telemetry.PanelsTelemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

import org.firstinspires.ftc.teamcode.Util.IfElseCommand;
import org.firstinspires.ftc.teamcode.Util.Poses;
import org.firstinspires.ftc.teamcode.Util.Timer;
import org.firstinspires.ftc.teamcode.Util.UniConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.Constants;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.commands.utility.LambdaCommand;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.ftc.ActiveOpMode;
import dev.nextftc.hardware.impl.MotorEx;

@Configurable
//TODO: Tune all constants to ensure correctness awesome - prob not needed
public class MecDriveSubsystem implements Subsystem {
    //Class variables
    JoinedTelemetry telemetry;
    UniConstants.teamColor color = UniConstants.teamColor.BLUE;
    public static boolean debug = false;
    private Follower follower;
    private double distanceToGoal = 0;
    private boolean driving = false;


    //For calculated turret angle
    private static double goalAngle = 0;
    private static double obeliskAngle = 0;

    private static final MotorEx fl = new MotorEx(UniConstants.DRIVE_FRONT_LEFT_STRING).floatMode().reversed();
    private static final MotorEx fr = new MotorEx(UniConstants.DRIVE_FRONT_RIGHT_STRING).floatMode().reversed();
    private static final MotorEx bl = new MotorEx(UniConstants.DRIVE_BACK_LEFT_STRING).floatMode();
    private static final MotorEx br = new MotorEx(UniConstants.DRIVE_BACK_RIGHT_STRING).floatMode().reversed();

    public static MecDriveSubsystem INSTANCE = new MecDriveSubsystem();



    public MecDriveSubsystem(){}

    @Override
    public void initialize(){
        color = Robot.color;
        telemetry = new JoinedTelemetry(ActiveOpMode.telemetry(), PanelsTelemetry.INSTANCE.getFtcTelemetry());
        follower = PedroComponent.follower();


    }


    @Override
    public void periodic(){
        //driving = !MecDriveSubsystem.INSTANCE.getFollower().getTeleopDriveVector().equals(new Vector(0, 0));
        updateDistanceAndAngle();
        //follower.update();
    }

    public void startTele(){
        follower.startTeleopDrive();
        follower.update();
    }
    public void updateTeleop(double forward, double strafe, double rotation, boolean botCentric){
        if(follower != null) {
            follower.setTeleOpDrive(forward, strafe, rotation, botCentric);
            follower.update();
        } else {
            follower = Constants.createFollower(ActiveOpMode.hardwareMap());
        }
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


    public Command FollowPath(PathChain path, boolean holdEnd){
        return new LambdaCommand("Follow Path Hold End: " + holdEnd)
                .setStart(() -> {MecDriveSubsystem.INSTANCE.getFollower().followPath(path, holdEnd); Robot.automatedDrive = true;})
                .setIsDone(() -> follower.getPose().roughlyEquals(path.endPose(), 1))
                .setInterruptible(false)
                .requires(MecDriveSubsystem.INSTANCE);
    }

    public Command FollowPath(PathChain path, boolean holdEnd, double maxPower){
        return new LambdaCommand("Follow Path Hold End: " + holdEnd)
                .setStart(() -> {MecDriveSubsystem.INSTANCE.getFollower().followPath(path, maxPower, holdEnd); Robot.automatedDrive = true;})
                .setIsDone(() -> follower.getPose().roughlyEquals(path.endPose(), 3) || driving)
                .setInterruptible(false)
                .requires(MecDriveSubsystem.INSTANCE);
    }



    public Command FollowPathTime(PathChain path, boolean holdEnd, double seconds){
        Timer timer = new Timer();
        return new LambdaCommand("Follow Path Time Hold End: " + holdEnd)
                .setStart(() -> {MecDriveSubsystem.INSTANCE.getFollower().followPath(path, holdEnd); timer.reset(); Robot.automatedDrive = true;})
                .setIsDone(() -> timer.getTimeSeconds() > seconds)
                .setInterruptible(false)
                .requires(MecDriveSubsystem.INSTANCE);
    }

    public Command FollowPathTime(PathChain path, boolean holdEnd, double maxPower, double seconds){
        Timer timer = new Timer();
        return new LambdaCommand("Follow Path Time Hold End: " + holdEnd)
                .setStart(() -> {MecDriveSubsystem.INSTANCE.getFollower().followPath(path, maxPower, holdEnd); timer.reset(); Robot.automatedDrive = true;})
                .setIsDone(() -> timer.getTimeSeconds() > seconds)
                .setInterruptible(false)
                .requires(MecDriveSubsystem.INSTANCE);
    }

    public Command PushForward(double power, double distance, boolean useIntake){
        follower.startTeleopDrive();
        Pose startDistance = follower.getPose();
        Robot.automatedDrive = true;
        return new SequentialGroup(
                new IfElseCommand(() -> useIntake, IntakeSortingSubsystem.INSTANCE.runActive(), IntakeSortingSubsystem.INSTANCE.stopActive()),
                new LambdaCommand()
                        .setStart(() -> {
                            setAllMotorPower(power);
                        })

                        .setIsDone(() -> Math.abs(follower.getPose().distanceFrom(startDistance)) >= distance || driving)
                        .setStop(inter -> {
                            setAllMotorPower(0);
                        })
                        .requires(INSTANCE, IntakeSortingSubsystem.INSTANCE)
                        .setInterruptible(true)
        );
    }

    public Command TurnTo(double degree){
        return new LambdaCommand()
                .setStart(() -> {follower.turnTo(Math.toRadians(degree)); Robot.automatedDrive = true;})
                .setIsDone(() -> Math.abs(Math.toDegrees(follower.getPose().getHeading()) - degree) < 1d)
                .setStop( interrupted -> {
                    if(Robot.inTeleop){
                        follower.startTeleopDrive();
                    }
                });

    }

    public void setAllMotorPower(double power){
        power = Math.max(-1, Math.min(1, power));
        fl.setPower(power);
        fr.setPower(power);
        bl.setPower(power);
        br.setPower(power);
    }

    public double getDistanceToGoal(){
        return distanceToGoal;
    }

    public double getGoalAngle(){
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

    public void setColor(UniConstants.teamColor color){
        this.color = color;
    }
    public PathChain createParkPath(){
        return follower.pathBuilder()
                .addPath((follower.getPose().getX() >= 72) ?
                        new BezierCurve(follower.getPose(), color == UniConstants.teamColor.BLUE ? Poses.bluePark : Poses.redPark, color == UniConstants.teamColor.BLUE ? Poses.blueParkCP : Poses.redParkCP) :
                        new BezierLine(follower.getPose(), color == UniConstants.teamColor.BLUE ? Poses.bluePark : Poses.redPark))
                .setConstantHeadingInterpolation(Math.toRadians(90)).build();
    }


    public PathChain createShootingPath(){
        return follower.pathBuilder()
                .addPath(new BezierLine(follower.getPose(), color == UniConstants.teamColor.BLUE ? Poses.blueShortScore : Poses.redShortScore))
                .setConstantHeadingInterpolation(color == UniConstants.teamColor.BLUE ? Poses.blueShortScore.getHeading() : Poses.redShortScore.getHeading()).build();
    }

    public void setStartPose(Pose pose){
        if(follower.atPose(new Pose(0, 0), 1, 1)){
            follower.setStartingPose(pose);
        }
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

    public void setTelemetry(JoinedTelemetry telemetry){
        this.telemetry = telemetry;
    }



}
