package org.firstinspires.ftc.teamcode.OpModes;

import static dev.nextftc.extensions.pedro.PedroComponent.follower;

import org.firstinspires.ftc.teamcode.Subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.OuttakeSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.RobotSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.Slots.MainSlot;
import org.firstinspires.ftc.teamcode.Subsystems.SortingSubsystem;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.Constants;

import dev.nextftc.core.commands.CommandManager;
import dev.nextftc.core.components.BindingsComponent;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.ftc.Gamepads;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;

public class TeleOp extends NextFTCOpMode {

    {
        addComponents(
                new PedroComponent(Constants::createFollower),
                BulkReadComponent.INSTANCE,
                new SubsystemComponent(RobotSubsystem.INSTANCE),
                BindingsComponent.INSTANCE
                
        );
    }


    @Override
    public void onWaitForStart() {
        if (gamepad1.a) {
            RobotSubsystem.allianceColor = RobotSubsystem.AllianceColor.RED;
        } else if (gamepad1.b) {
            RobotSubsystem.allianceColor = RobotSubsystem.AllianceColor.BLUE;
        }


        telemetry.addLine("CHANGE THIS IF NEED BE!!!! ");
        telemetry.addLine("Circle for Blue, X for Red ");
        telemetry.addData("Current Team Color ", RobotSubsystem.allianceColor);
    }


    @Override
    public void onStartButtonPressed() {
        follower().setStartingPose(RobotSubsystem.previousPose);
        follower().startTeleopDrive();
        createBindings();

    }

    @Override
    public void onUpdate() {
        RobotSubsystem.previousPose = follower().getPose();

        //    private final boolean botCentric = true;
        boolean isSlowed = gamepad1.left_bumper;

        //Spin active forward
        if (gamepad1.right_trigger > 0) {
            IntakeSubsystem.INSTANCE.setActiveState(IntakeSubsystem.IntakeState.IN);
        } //Reverse Active and Spin
        else if (gamepad1.left_trigger > 0) {
            IntakeSubsystem.INSTANCE.setActiveState(IntakeSubsystem.IntakeState.OUT);
        } else {
            IntakeSubsystem.INSTANCE.setActiveState(IntakeSubsystem.IntakeState.OFF);
        }

        //Kill button
        if ((gamepad1.yWasPressed())) {
            CommandManager.INSTANCE.cancelAll();
            SortingSubsystem.INSTANCE.SetAllSlotState(MainSlot.ServoState.DOWN).schedule();
            follower().startTeleopDrive();
        }

        //Driver controlled

        follower().setTeleOpDrive(
                -gamepad1.left_stick_y * (isSlowed ? .25 : 1),
                -gamepad1.left_stick_x * (isSlowed ? .25 : 1),
                -gamepad1.right_stick_x * (isSlowed ? .25 : 1),
                true
        );




    }

    private void createBindings() {


        //Toggle things based on dpad
        Gamepads.gamepad1().dpadUp().whenBecomesTrue(OuttakeSubsystem.INSTANCE.SetTurretState(OuttakeSubsystem.TurretState.FORWARD));
        Gamepads.gamepad1().dpadLeft().whenBecomesTrue(OuttakeSubsystem.INSTANCE.SetTurretState(OuttakeSubsystem.TurretState.GOAL));
        //Gamepads.gamepad1().dpadRight().whenBecomesTrue(TurretSubsystem.INSTANCE::init);
        //Gamepads.gamepad1().dpadDown().whenBecomesTrue(SlotsSubsystem.INSTANCE.SetAllSlotState(Slot.ServoState.DOWN));


        //Face buttons
//        Gamepads.gamepad1().a().whenBecomesTrue(TurretSubsystem.INSTANCE.SetFlywheelState(TurretSubsystem.FlywheelState.SHORT));
//        Gamepads.gamepad1().b().whenBecomesTrue(TurretSubsystem.INSTANCE.SetFlywheelState(TurretSubsystem.FlywheelState.OFF));
//        Gamepads.gamepad1().x().whenBecomesTrue(TurretSubsystem.INSTANCE.SetFlywheelState(TurretSubsystem.FlywheelState.FAR));

        //Shooting command
        Gamepads.gamepad1().rightBumper().whenBecomesTrue(SortingSubsystem.INSTANCE.Shoot());


    }
    
}
