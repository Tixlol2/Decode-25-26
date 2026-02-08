package org.firstinspires.ftc.teamcode.OpModes;

import static dev.nextftc.extensions.pedro.PedroComponent.follower;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.OuttakeSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.RobotSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.Slots.MainSlot;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.Constants;

import dev.nextftc.core.commands.CommandManager;
import dev.nextftc.core.components.BindingsComponent;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.ftc.Gamepads;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;

@TeleOp(name = "Tele", group = "Main")
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

        if(resetTurret){
            OuttakeSubsystem.INSTANCE.resetTurret();
            resetTurret = false;
        }

        follower().setStartingPose(RobotSubsystem.INSTANCE.getPreviousPose());
        follower().startTeleopDrive();
        createBindings();

    }

    @Override
    public void onUpdate() {
        RobotSubsystem.INSTANCE.setPreviousPose(follower().getPose());

        //Intake control

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


        

        //Driver controlled

        follower().setTeleOpDrive(
                -gamepad1.left_stick_y * (isSlowed ? .25 : 1),
                -gamepad1.left_stick_x * (isSlowed ? .25 : 1),
                -gamepad1.right_stick_x * (isSlowed ? .25 : 1),
                true
        );

        telemetry.addData("Turret RPM: ", OuttakeSubsystem.INSTANCE.getCurrentVelocityRPM());
        telemetry.addData("Hood Taqrget Pos: ", OuttakeSubsystem.INSTANCE.getHoodTarget());
        telemetry.addData("Pattern: ", RobotSubsystem.INSTANCE.getPattern());
        telemetry.addData("Command Manager: ", CommandManager.INSTANCE.snapshot());
    }

    private void createBindings() {


        //Toggle things based on dpad
        Gamepads.gamepad1().dpadUp().whenBecomesTrue(OuttakeSubsystem.INSTANCE.SetTurretState(OuttakeSubsystem.TurretState.FORWARD));
        Gamepads.gamepad1().dpadLeft().whenBecomesTrue(OuttakeSubsystem.INSTANCE.SetTurretState(OuttakeSubsystem.TurretState.GOAL));


        //Face buttons
        Gamepads.gamepad1().a().whenBecomesTrue(OuttakeSubsystem.INSTANCE.SetFlywheelState(OuttakeSubsystem.FlywheelState.MEDIUM));
        Gamepads.gamepad1().b().whenBecomesTrue(OuttakeSubsystem.INSTANCE.SetFlywheelState(OuttakeSubsystem.FlywheelState.OFF));
        Gamepads.gamepad1().x().whenBecomesTrue(OuttakeSubsystem.INSTANCE.SetFlywheelState(OuttakeSubsystem.FlywheelState.FULL));
        Gamepads.gamepad1().y().whenBecomesTrue(() -> {
            RobotSubsystem.INSTANCE.resetPattern();
            CommandManager.INSTANCE.cancelAll();
            RobotSubsystem.INSTANCE.SetAllSlotState(MainSlot.ServoState.DOWN).schedule();
            follower().startTeleopDrive();
        });

        //Shooting command
        Gamepads.gamepad1().rightBumper().whenBecomesTrue(RobotSubsystem.INSTANCE.Shoot());


    }

}
