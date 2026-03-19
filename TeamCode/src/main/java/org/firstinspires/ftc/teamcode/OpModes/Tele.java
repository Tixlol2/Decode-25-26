package org.firstinspires.ftc.teamcode.OpModes;

import static dev.nextftc.extensions.pedro.PedroComponent.follower;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.OuttakeSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.RobotSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.Slots.MainSlot;
import org.firstinspires.ftc.teamcode.Subsystems.VisionSubsystemLL;
import org.firstinspires.ftc.teamcode.Util.Poses;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.Constants;

import dev.nextftc.core.commands.CommandManager;
import dev.nextftc.core.components.BindingsComponent;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.ftc.Gamepads;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;

@TeleOp(name = "Tele", group = "Main")
@Configurable
public class Tele extends NextFTCOpMode {

    {
        addComponents(
                new PedroComponent(Constants::createFollower),
                BulkReadComponent.INSTANCE,
                new SubsystemComponent(RobotSubsystem.INSTANCE),
                BindingsComponent.INSTANCE
        );
    }

    private boolean resetTurret = false;



    public static boolean farZone = false;

    public static Pose autoPose = new Pose();

    @Override
    public void onWaitForStart() {
        if (gamepad1.a) {
            RobotSubsystem.INSTANCE.setAllianceColor(RobotSubsystem.AllianceColor.RED);
        } else if (gamepad1.b) {
            RobotSubsystem.INSTANCE.setAllianceColor(RobotSubsystem.AllianceColor.BLUE);
        }

        if(gamepad1.triangle){
            resetTurret = true;
        }

        if(gamepad1.square){
            resetTurret = false;
        }

        if(gamepad1.dpad_up){
            farZone = false;
        } else if (gamepad1.dpad_down){
            farZone = true;
        }

        telemetry.addLine("Press Tri to Reset Turret on Start");
        telemetry.addLine("Press Square to NOT Reset Turret on Start");
        telemetry.addData("Resetting? ", resetTurret);
        telemetry.addLine();
        telemetry.addLine("CHANGE THIS IF NEED BE!!!! ");
        telemetry.addLine("Circle for Blue, X for Red ");
        telemetry.addData("Current Team Color ", RobotSubsystem.INSTANCE.getAllianceColor());
    }


    @Override
    public void onStartButtonPressed() {

        if(resetTurret) {
            OuttakeSubsystem.INSTANCE.resetTurret();
            resetTurret = false;
            if (!farZone) {
                follower().setStartingPose(RobotSubsystem.INSTANCE.getAllianceColor() == RobotSubsystem.AllianceColor.BLUE ? Poses.blueCloseStart : Poses.redCloseStart);
            } else {
                follower().setStartingPose(RobotSubsystem.INSTANCE.getAllianceColor() == RobotSubsystem.AllianceColor.BLUE ? Poses.blueFarStart : Poses.redFarStart);
            }
        } else {
            follower().setStartingPose(NewAuto.prevPose);
        }

        RobotSubsystem.inTele = true;
        follower().startTeleopDrive();
        createBindings();

    }

    @Override
    public void onUpdate() {

        autoPose = PedroComponent.follower().getPose();

        //Intake control
        boolean isSlowed = gamepad1.left_bumper;
        RobotSubsystem.INSTANCE.setPatternShiftingEnabled(gamepad1.left_bumper);

        //Spin active forward
        if (gamepad1.right_trigger > 0) {
            IntakeSubsystem.INSTANCE.setActiveState(IntakeSubsystem.IntakeState.IN);
        } //Reverse Active and Spin
        else if (gamepad1.left_trigger > 0) {
            IntakeSubsystem.INSTANCE.setActiveState(IntakeSubsystem.IntakeState.OUT);
        } else {
            IntakeSubsystem.INSTANCE.setActiveState(IntakeSubsystem.IntakeState.OFF);
        }

        

        //Driver controlled
        follower().setTeleOpDrive(
                -gamepad1.left_stick_y * (isSlowed ? .25 : 1),
                -gamepad1.left_stick_x * (isSlowed ? .25 : 1),
                -gamepad1.right_stick_x * (isSlowed ? .25 : 1),
                true
        );

        OuttakeSubsystem.INSTANCE.sendTelemetry();
//        telemetry.addData("Command Manager: ", CommandManager.INSTANCE.snapshot());
        telemetry.addData("Pose X: ", PedroComponent.follower().getPose().getX());
        telemetry.addData("Pose Y: ", PedroComponent.follower().getPose().getY());
        telemetry.addData("Pose H: ", Math.toDegrees(PedroComponent.follower().getPose().getHeading()));
        telemetry.addData("Distance to Goal Odo: ", RobotSubsystem.INSTANCE.getDistanceToGoalInches());
        telemetry.addData("Distance to Goal LL: ", VisionSubsystemLL.INSTANCE.getDistanceToGoal());
    }

    private void createBindings() {


        //Toggle things based on dpad
        Gamepads.gamepad2().dpadUp().whenBecomesTrue(OuttakeSubsystem.addUserAdded());
        Gamepads.gamepad2().dpadLeft().whenBecomesTrue(OuttakeSubsystem.INSTANCE.SetTurretState(OuttakeSubsystem.TurretState.GOAL));
        Gamepads.gamepad2().dpadDown().whenBecomesTrue(OuttakeSubsystem.subUserAdded());
        Gamepads.gamepad2().dpadRight().whenBecomesTrue(OuttakeSubsystem.INSTANCE.SetTurretState(OuttakeSubsystem.TurretState.FORWARD));

        Gamepads.gamepad2().leftStickButton().whenBecomesTrue(OuttakeSubsystem.INSTANCE.SetTurretState(OuttakeSubsystem.TurretState.LIME));

        //Face buttons
        Gamepads.gamepad2().a().whenBecomesTrue(OuttakeSubsystem.INSTANCE.SetFlywheelState(OuttakeSubsystem.FlywheelState.INTERPOLATED));
        Gamepads.gamepad2().rightStickButton().whenBecomesTrue(OuttakeSubsystem.INSTANCE.SetFlywheelState(OuttakeSubsystem.FlywheelState.REACTIVE));
        Gamepads.gamepad2().b().whenBecomesTrue(OuttakeSubsystem.INSTANCE.SetFlywheelState(OuttakeSubsystem.FlywheelState.OFF));
        Gamepads.gamepad2().x().whenBecomesTrue(OuttakeSubsystem.INSTANCE.SetFlywheelState(OuttakeSubsystem.FlywheelState.LAZY));
        Gamepads.gamepad2().y().whenBecomesTrue(() -> {
            RobotSubsystem.INSTANCE.resetPattern();
            CommandManager.INSTANCE.cancelAll();
            RobotSubsystem.INSTANCE.SetAllSlotState(MainSlot.ServoState.DOWN).schedule();
            follower().startTeleopDrive();
        });

        //Shooting command
        Gamepads.gamepad1().rightBumper().whenBecomesTrue(RobotSubsystem.INSTANCE.Shoot());
        Gamepads.gamepad2().rightBumper().whenBecomesTrue(RobotSubsystem.INSTANCE.AutoShoot());


    }

}
