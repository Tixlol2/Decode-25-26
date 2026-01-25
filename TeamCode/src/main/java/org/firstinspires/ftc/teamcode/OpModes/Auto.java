//package org.firstinspires.ftc.teamcode.OpModes;
//
//import static dev.nextftc.extensions.pedro.PedroComponent.follower;
//
//import com.bylazar.telemetry.JoinedTelemetry;
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//
//import org.firstinspires.ftc.teamcode.Util.Subsystems.Robot;
//import org.firstinspires.ftc.teamcode.Util.Subsystems.TurretSubsystem;
//import org.firstinspires.ftc.teamcode.pedroPathing.constants.Constants;
//
//import dev.nextftc.core.commands.Command;
//import dev.nextftc.core.commands.utility.InstantCommand;
//import dev.nextftc.core.components.SubsystemComponent;
//import dev.nextftc.extensions.pedro.PedroComponent;
//import dev.nextftc.ftc.NextFTCOpMode;
//import dev.nextftc.ftc.components.BulkReadComponent;
//
//@Autonomous(name = "Auto", group = "Main")
//public class Auto extends NextFTCOpMode {
//
//
//    JoinedTelemetry joinedTelemetry;
//    Paths paths;
//    private int pathState = 0;
//    private int oldPathState = -1;
//
//    {
//        addComponents(
//                new PedroComponent(Constants::createFollower),
//                new SubsystemComponent(Robot.INSTANCE),
//                BulkReadComponent.INSTANCE
//        ); //Subsystems
//    }
//
//    @Override
//    public void onInit() {
//        joinedTelemetry = Robot.INSTANCE.getJoinedTelemetry();
//        Robot.inTeleop = false;
//        TurretSubsystem.INSTANCE.init();
//        paths = new Paths();
//        paths.setPathsClose6();
//    }
//
//    @Override
//    public void onWaitForStart() {
//        if (gamepad1.a) {
//            Robot.color = Robot.teamColor.RED;
//        } else if (gamepad1.b) {
//            Robot.color = Robot.teamColor.BLUE;
//        }
//
//        if (gamepad1.dpad_up) {
//            paths.setPathsClose6();
//        } else if (gamepad1.dpad_down){
//            paths.setPathsClose9();
//        }
//
//        joinedTelemetry.addLine("CHANGE THIS IF NEED BE!!!! ");
//        joinedTelemetry.addLine("Circle for Blue, X for Red ");
//        joinedTelemetry.addData("Current Team Color ", Robot.color);
//        joinedTelemetry.addData("Current Path ", paths.name);
//    }
//
//    @Override
//    public void onStartButtonPressed() {
//        follower().setStartingPose(paths.startingPose);
//        Robot.INSTANCE.setGlobalColor();
//        setPathState(0);
//    }
//
//    @Override
//    public void onUpdate() {
//        Robot.previousPose = follower().getPose();
//        if (pathState != oldPathState) {
//            oldPathState = pathState;
//            pathUpdate();
//        }
//    }
//
//
//    public void setPathState(int state) {
//        if (paths.numCommands >= state) {
//            pathState = state;
//        } else {
//            pathState = -1;
//        }
//
//    }
//
//    public Command SetPathState(int state) {
//        return new InstantCommand(() -> setPathState(state));
//    }
//
//    public void pathUpdate() {
//
//        switch (pathState) {
//            case -1:
//                break;
//            case 0:
//                paths.command1.then(SetPathState(1)).schedule();
//                break;
//            case 1:
//                paths.command2.then(SetPathState(2)).schedule();
//                break;
//            case 2:
//                paths.command3.then(SetPathState(3)).schedule();
//                break;
//            case 3:
//                paths.command4.then(SetPathState(4)).schedule();
//                break;
//            case 4:
//                paths.command5.then(SetPathState(5)).schedule();
//                break;
//            case 5:
//                paths.command6.then(SetPathState(6)).schedule();
//                break;
//            case 6:
//                paths.command7.then(SetPathState(7)).schedule();
//                break;
//
//        }
//
//
//    }
//
//}
