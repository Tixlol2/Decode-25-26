package org.firstinspires.ftc.teamcode.OpModes;

import com.bylazar.telemetry.JoinedTelemetry;
import com.bylazar.telemetry.PanelsTelemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Util.Subsystems.MecDriveSubsystem;
import org.firstinspires.ftc.teamcode.Util.Subsystems.OuttakeSubsystem;
import org.firstinspires.ftc.teamcode.Util.Subsystems.RotaryIntakeSubsystem;
import org.firstinspires.ftc.teamcode.Util.Timer;
import org.firstinspires.ftc.teamcode.Util.UniConstants;

import dev.nextftc.ftc.NextFTCOpMode;

@Autonomous(name = "far auto")
public class FarAuto extends NextFTCOpMode {

    {
        addComponents();
    }

    JoinedTelemetry joinedTelemetry;
    UniConstants.teamColor color = UniConstants.teamColor.BLUE;

    Timer pathTimer = new Timer();

    int pathState = -1;
    PathChain path1, path2;


    private static RotaryIntakeSubsystem rotaryIntake;
    private static OuttakeSubsystem outtake;
    private static MecDriveSubsystem mecDrive;

    public Servo ballServo;

    @Override
    public void onInit() {
        joinedTelemetry = new JoinedTelemetry(telemetry, PanelsTelemetry.INSTANCE.getFtcTelemetry());
//        vision = new BetterVisionTM(hardwareMap, joinedTelemetry, logState);
        rotaryIntake = new RotaryIntakeSubsystem(hardwareMap, joinedTelemetry, color);
        outtake = new OuttakeSubsystem(hardwareMap, joinedTelemetry, color);
        mecDrive = new MecDriveSubsystem(hardwareMap, joinedTelemetry, color);

        ballServo = hardwareMap.get(Servo.class, UniConstants.BALL_SERVO_STRING);


        //mecDrive.resetPinpoint();
        outtake.resetMotors();

//        buildPaths(mecDrive.getFollower());

    }

    @Override
    public void onWaitForStart() {
        if (gamepad1.a) {
            color = UniConstants.teamColor.RED;
        } else if (gamepad1.b) {
            color = UniConstants.teamColor.BLUE;
        }

        joinedTelemetry.addLine("CHANGE THIS IF NEED BE!!!! ");
        joinedTelemetry.addLine("B for Blue, A for Red ");
        joinedTelemetry.addData("Current Team Color ", color);
        joinedTelemetry.update();
    }

    @Override
    public void onStartButtonPressed() {

        buildPaths(mecDrive.getFollower());
        rotaryIntake.setColor(color);
        outtake.setColor(color);
        mecDrive.setColor(color);
        mecDrive.setPose(color == UniConstants.teamColor.BLUE ? new Pose(49, 8.5, Math.toRadians(90)) : new Pose(95, 8.5, Math.toRadians(90)));

        setPathState(0);

    }

    @Override
    public void onUpdate(){
        mecDrive.periodic();
        outtake.periodic();
        rotaryIntake.periodic();

        mecDrive.sendTelemetry(UniConstants.loggingState.ENABLED);
        telemetry.addData("Path State ", pathState);
        telemetry.addData("Follower Is Busy ", mecDrive.getFollower().isBusy());
        telemetry.update();
        autoUpdate();
    }

    public void autoUpdate(){
        switch(pathState){
            case 0:
                outtake.setLauncherTargetVelo(2200);
                mecDrive.getFollower().followPath(path1);
                setPathState(1);
                break;
            case 1:
                if(!mecDrive.getFollower().isBusy() && pathTimer.getTimeSeconds() > 7.5){

                    if(pathTimer.getTimeSeconds() > 8 && pathTimer.getTimeSeconds() < 10) {
                        rotaryIntake.enableActive();
//                    ballServo.setPosition(UniConstants.SERVO_OUTTAKE);
                        rotaryIntake.toggleServo();
                    }
                    else if(pathTimer.getTimeSeconds() > 10  && pathTimer.getTimeSeconds() < 12.5){
                        rotaryIntake.toggleServo();
                    }

                    else if(pathTimer.getTimeSeconds() > 12.5){
                        rotaryIntake.toggleServo();
                        setPathState(2);
                    }
                }
                break;
            case 2:
                if(pathTimer.getTimeSeconds() > 3.5){
                    rotaryIntake.disableActive();
//                    ballServo.setPosition(UniConstants.SERVO_INTAKE);
                    rotaryIntake.toggleServo();
                    mecDrive.getFollower().followPath(path2);
                    setPathState(3);
                }
                break;
            case 3:
                outtake.setLauncherPowerDebug(0);
                break;
        }
    }

    public void setPathState(int pathState) {
        this.pathState = pathState;
        pathTimer.reset();
    }

    public void buildPaths(Follower follower){
        if(color == UniConstants.teamColor.BLUE){
        path1 = follower.pathBuilder()
                .addPath(
                        // Line 1
                        new BezierLine(new Pose(49, 8.5), new Pose(45, 100))
                )
                .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(144))
                .build();

        path2 = follower.pathBuilder()
                .addPath(
                        new BezierLine(new Pose(45, 100), new Pose(37, 135))
                )
                .setLinearHeadingInterpolation(Math.toRadians(144), Math.toRadians(180))
                .build();
    }
        else if (color == UniConstants.teamColor.RED){
            path1 = follower.pathBuilder()
                    .addPath(
                            // Line 1
                            new BezierLine(new Pose(95, 8.5), new Pose(93, 105))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(36))
                    .build();

            path2 = follower.pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(93, 105), new Pose(96, 133))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(36), Math.toRadians(0))
                    .build();
        }

    }

}