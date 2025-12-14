package org.firstinspires.ftc.teamcode.OpModes;

import com.bylazar.telemetry.JoinedTelemetry;
import com.bylazar.telemetry.PanelsTelemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.OpModes.Util.BluePaths;
import org.firstinspires.ftc.teamcode.OpModes.Util.RedPaths;
import org.firstinspires.ftc.teamcode.Util.Subsystems.MecDriveSubsystem;
import org.firstinspires.ftc.teamcode.Util.Subsystems.OuttakeSubsystem;
import org.firstinspires.ftc.teamcode.Util.Subsystems.RotaryIntakeSubsystem;
import org.firstinspires.ftc.teamcode.Util.Timer;
import org.firstinspires.ftc.teamcode.Util.UniConstants;

import dev.nextftc.ftc.NextFTCOpMode;

@Autonomous(name = "good auto")
public class GoodAuton extends NextFTCOpMode {

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

    BluePaths bluePaths;
    RedPaths redPaths;

    @Override
    public void onInit() {
        joinedTelemetry = new JoinedTelemetry(telemetry, PanelsTelemetry.INSTANCE.getFtcTelemetry());
//        vision = new BetterVisionTM(hardwareMap, joinedTelemetry, logState);
        rotaryIntake = new RotaryIntakeSubsystem(hardwareMap, joinedTelemetry, color);
        outtake = new OuttakeSubsystem(hardwareMap, joinedTelemetry, color);
        mecDrive = new MecDriveSubsystem(hardwareMap, joinedTelemetry, color);

//        ballServo = hardwareMap.get(Servo.class, UniConstants.BALL_SERVO_STRING);

        bluePaths = new BluePaths(mecDrive.getFollower());
        redPaths = new RedPaths(mecDrive.getFollower());



        //mecDrive.resetPinpoint();
        outtake.resetMotors();

        //buildPaths(mecDrive.getFollower());


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
        mecDrive.setPose(color == UniConstants.teamColor.RED ? new Pose(122, 125, Math.toRadians(36)) : new Pose(22.000, 124.000, Math.toRadians(144)));
        rotaryIntake.toggleServo(RotaryIntakeSubsystem.servoState.INTAKE);

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
        telemetry.addData("Timer Seconds ", pathTimer.getTimeSeconds());
        telemetry.update();
        autoUpdate();
    }

    public void autoUpdate(){

        switch(pathState){
            case -1:
                //Do nothing state
                break;
            case 0:
                outtake.setLauncherTargetVelo(2200);
                mecDrive.getFollower().followPath(color == UniConstants.teamColor.BLUE ? bluePaths.Path1 : redPaths.Path1);
                setPathState(1);
                break;
            case 1:
                if(!mecDrive.getFollower().isBusy()){
                    if(pathTimer.getTimeSeconds() > 8 && pathTimer.getTimeSeconds() < 9.5) { //This timer is mainly for the launcher getting spun up
                        rotaryIntake.enableActive();
//                    ballServo.setPosition(UniConstants.SERVO_OUTTAKE);
                        rotaryIntake.toggleServo(RotaryIntakeSubsystem.servoState.OUTTAKE); //Up
                    }
                    else if(pathTimer.getTimeSeconds() > 9.5 && pathTimer.getTimeSeconds() < 10.5){
                        rotaryIntake.toggleServo(RotaryIntakeSubsystem.servoState.INTAKE); //Down
                    }

                    else if(pathTimer.getTimeSeconds() > 10.5){
                        rotaryIntake.toggleServo(RotaryIntakeSubsystem.servoState.OUTTAKE); //Up
                        setPathState(2);
                    }
                }
                break;
            case 2:
                if(pathTimer.getTimeSeconds() > 2){ //Unsure of why this is its own state?

//                    ballServo.setPosition(UniConstants.SERVO_INTAKE);
                    rotaryIntake.toggleServo(RotaryIntakeSubsystem.servoState.INTAKE); //Down
                    mecDrive.getFollower().followPath(color == UniConstants.teamColor.BLUE ? bluePaths.Path2 : redPaths.Path2);
                    setPathState(3);
                }
                break;
            case 3:
                //Get to intake position
                if(!mecDrive.getFollower().isBusy() ){
                    rotaryIntake.enableActive();
                    mecDrive.getFollower().followPath(color == UniConstants.teamColor.BLUE ? bluePaths.Path3 : redPaths.Path3);
                    setPathState(4);
                }
                break;
            case 4:
                //Intake all balls
                if(!mecDrive.getFollower().isBusy()){

                    mecDrive.getFollower().followPath(color == UniConstants.teamColor.BLUE ? bluePaths.Path4 : redPaths.Path4);
                    setPathState(5);
                }
                break;
            case 5:
                //Get to shooting location, and then shoot
                if(!mecDrive.getFollower().isBusy()){
                    if(pathTimer.getTimeSeconds() >= 3 && pathTimer.getTimeSeconds() < 5.5) {
                        rotaryIntake.enableActive();
//                    ballServo.setPosition(UniConstants.SERVO_OUTTAKE);
                        rotaryIntake.toggleServo(RotaryIntakeSubsystem.servoState.OUTTAKE); //Up
                    }
                    else if(pathTimer.getTimeSeconds() >= 5.5 && pathTimer.getTimeSeconds() < 8){
                        rotaryIntake.toggleServo(RotaryIntakeSubsystem.servoState.INTAKE); //Down
                    }

                    else if(pathTimer.getTimeSeconds() >= 8){
                        rotaryIntake.toggleServo(RotaryIntakeSubsystem.servoState.OUTTAKE); //Up
                        setPathState(6);
                    }
                }
                break;
            case 6:
                //Prepare Intake
                if(pathTimer.getTimeSeconds() > 2){
                    mecDrive.getFollower().followPath(color == UniConstants.teamColor.BLUE ? bluePaths.Path8 : redPaths.Path8);
                    setPathState(-1);
                }
                break;
            case 7:
                //Go to 2nd line and intake
                if(!mecDrive.getFollower().isBusy()){
                    rotaryIntake.toggleServo();
                    rotaryIntake.enableActive();
                    mecDrive.getFollower().followPath(color == UniConstants.teamColor.BLUE ? bluePaths.Path6 : redPaths.Path6);
                    setPathState(8);
                }
                break;
            case 8:
                if(!mecDrive.getFollower().isBusy()){
                    if(pathTimer.getTimeSeconds() > 3) {
                        rotaryIntake.enableActive();
//                    ballServo.setPosition(UniConstants.SERVO_OUTTAKE);
                        rotaryIntake.toggleServo(RotaryIntakeSubsystem.servoState.OUTTAKE); //Up
                    }
                    else if(pathTimer.getTimeSeconds() > 5.5){
                        rotaryIntake.toggleServo(RotaryIntakeSubsystem.servoState.INTAKE); //Down
                    }

                    else if(pathTimer.getTimeSeconds() > 8){
                        rotaryIntake.toggleServo(RotaryIntakeSubsystem.servoState.OUTTAKE); //Up
                        setPathState(9);
                    }
                }
                break;
            case 9:
                if(pathTimer.getTimeSeconds() > 3){
                    outtake.setLauncherTargetVelo(0);
                    rotaryIntake.toggleServo();
                    rotaryIntake.disableActive();
                    mecDrive.getFollower().followPath(color == UniConstants.teamColor.BLUE ? bluePaths.Path7 : redPaths.Path7);
                    setPathState(-1);
                }
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
                        new BezierLine(new Pose(22.000, 124.000), new Pose(45, 100))
                )
                .setConstantHeadingInterpolation(Math.toRadians(144))
                .build();

        path2 = follower.pathBuilder()
                .addPath(
                        new BezierLine(new Pose(45, 10), new Pose(37, 135))
                )
                .setLinearHeadingInterpolation(Math.toRadians(144), Math.toRadians(180))
                .build();
    }
        else if (color == UniConstants.teamColor.RED){
            path1 = follower.pathBuilder()
                    .addPath(
                            // Line 1
                            new BezierLine(new Pose(122, 125.000), new Pose(93, 105))
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(36))
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
