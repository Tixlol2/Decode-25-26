package org.firstinspires.ftc.teamcode.OpModes;


import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.JoinedTelemetry;
import com.bylazar.telemetry.PanelsTelemetry;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Util.Poses;
import org.firstinspires.ftc.teamcode.Util.Subsystems.IntakeSortingSubsystem;
import org.firstinspires.ftc.teamcode.Util.Subsystems.MecDriveSubsystem;
import org.firstinspires.ftc.teamcode.Util.Subsystems.Robot;
import org.firstinspires.ftc.teamcode.Util.Subsystems.TurretSubsystem;
import org.firstinspires.ftc.teamcode.Util.Timer;
import org.firstinspires.ftc.teamcode.Util.UniConstants;

import dev.nextftc.core.commands.CommandManager;
import dev.nextftc.core.components.BindingsComponent;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.ftc.ActiveOpMode;
import dev.nextftc.ftc.Gamepads;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;


//Written by Noah Nottingham - 6566 Circuit Breakers

@TeleOp(name = "Functional Teleop", group = "Main") //The name and group
@Configurable
public class NextFTCTeleop extends NextFTCOpMode {


    private boolean isSlowed = false;

    private double distanceToGoalInMeters = 0.0;

    JoinedTelemetry joinedTelemetry;
    private boolean botCentric = true;


    private boolean enableRumble = false;

    Timer driverTimer = new Timer();
    Timer shootTimer = new Timer();
    Timer rumblingTimer = new Timer();

    {
        addComponents(
                new SubsystemComponent(Robot.INSTANCE),
                BulkReadComponent.INSTANCE,
                BindingsComponent.INSTANCE
        ); //Subsystems
    }

    public static Pose startPose = Poses.blueGoalTopStartFacing;


    //CommandManager manager;

    @Override
    public void onInit() {
        joinedTelemetry = new JoinedTelemetry(ActiveOpMode.telemetry(), PanelsTelemetry.INSTANCE.getFtcTelemetry());


        Robot.inTeleop = true;
        Robot.INSTANCE.initialize();
    }

    @Override
    public void onWaitForStart() {
        if (gamepad1.a) {
            Robot.color = UniConstants.teamColor.RED;
        } else if (gamepad1.b) {
            Robot.color = UniConstants.teamColor.BLUE;
        }

        joinedTelemetry.addLine("CHANGE THIS IF NEED BE!!!! ");
        joinedTelemetry.addLine("Circle for Blue, X for Red ");
        joinedTelemetry.addData("Current Team Color ", Robot.color);
        joinedTelemetry.update();
    }


    @Override
    public void onStartButtonPressed() {
        MecDriveSubsystem.INSTANCE.getFollower().setPose(startPose);
        TurretSubsystem.INSTANCE.setTargetVelocityTicks(0);
        MecDriveSubsystem.INSTANCE.startTele();
        //createBindings();
    }

    @Override
    public void onUpdate() {


        isSlowed = gamepad1.left_bumper;

        //Spin active forward
        if (gamepad1.right_trigger > 0) {
            IntakeSortingSubsystem.INSTANCE.forwardIntake();
        } //Reverse Active and Spin
        else if (gamepad1.left_trigger > 0) {
            IntakeSortingSubsystem.INSTANCE.reverseIntake();
        }
        else {
            IntakeSortingSubsystem.INSTANCE.disableActive();
        }

        if(gamepad1.a){
            TurretSubsystem.INSTANCE.setMotorPower(.67);
        }

        if(gamepad1.b){
            TurretSubsystem.INSTANCE.setMotorPower(0);
        }

        if(gamepad1.x){
            TurretSubsystem.INSTANCE.setMotorPower(1);
        }

        if(gamepad1.dpad_left){enableRumble = true;}
        if(gamepad1.dpad_right){enableRumble = false;}
        if(gamepad1.dpad_up){Robot.turretForward = true;}
        if(gamepad1.dpad_down){Robot.turretForward = false;}

        //Able to switch between driver and robot centric
        if(gamepad1.y && driverTimer.getTimeSeconds() > .5){
            botCentric = !botCentric;
            driverTimer.reset();
        }

        if(gamepad1.right_bumper && shootTimer.getTimeSeconds() > 3) {
            Robot.INSTANCE.ShootWait(.5).schedule();
            shootTimer.reset();
        }


        if(IntakeSortingSubsystem.INSTANCE.shouldRumble() && rumblingTimer.getTimeSeconds() > 3 && enableRumble){
            gamepad1.rumble(1000);
            rumblingTimer.reset();
        }




        //TODO: Still have to integrate look up table or linreg for power as a function of distance


        MecDriveSubsystem.INSTANCE.updateTeleop(
                -gamepad1.left_stick_y * (isSlowed ? .25 : 1), //Forward/Backward
                -gamepad1.left_stick_x * (isSlowed ? .25 : 1), //Left/Right Rotation
                -gamepad1.right_stick_x * (isSlowed ? .25 : 1), //Left/Right Strafe
                botCentric
        );






        joinedTelemetry.addData("Bot Centric: ", botCentric);
        joinedTelemetry.addData("Current Commands: ", CommandManager.INSTANCE.snapshot());
        TurretSubsystem.INSTANCE.sendTelemetry(UniConstants.loggingState.ENABLED);
        MecDriveSubsystem.INSTANCE.sendTelemetry(UniConstants.loggingState.ENABLED);
        joinedTelemetry.update();

    }

    void createBindings(){

        //Disable active when triggers not held down
        Gamepads.gamepad1().rightTrigger().atMost(.1).and(Gamepads.gamepad1().leftTrigger().atMost(.1)).whenFalse(IntakeSortingSubsystem.INSTANCE::disableActive);

        //Run active in direction based on trigger
        Gamepads.gamepad1().rightTrigger().greaterThan(0).whenTrue(
                () -> {
                    IntakeSortingSubsystem.INSTANCE.enableActive();
                    IntakeSortingSubsystem.INSTANCE.forwardIntake();
                });

        Gamepads.gamepad1().leftTrigger().greaterThan(0).whenTrue(
                () -> {
                    IntakeSortingSubsystem.INSTANCE.enableActive();
                    IntakeSortingSubsystem.INSTANCE.reverseIntake();
                });

        //Toggle things based on dpad
        Gamepads.gamepad1().dpadUp().toggleOnBecomesTrue().whenBecomesTrue(() -> {Robot.turretForward = !Robot.turretForward;});
        Gamepads.gamepad1().dpadLeft().toggleOnBecomesTrue().whenBecomesTrue(() -> {enableRumble = !enableRumble;});

        //Face buttons
        Gamepads.gamepad1().a().whenBecomesTrue(() -> {TurretSubsystem.INSTANCE.setMotorPower(.6);});
        Gamepads.gamepad1().b().whenBecomesTrue(() -> {TurretSubsystem.INSTANCE.setMotorPower(0);});
        Gamepads.gamepad1().x().whenBecomesTrue(() -> {TurretSubsystem.INSTANCE.setMotorPower(1);});
        Gamepads.gamepad1().y().toggleOnBecomesTrue().whenBecomesTrue(() -> {botCentric = !botCentric;});

        //Shooting command
        Gamepads.gamepad1().rightBumper().whenBecomesTrue(() -> {
           Robot.INSTANCE.ShootWait(.5).schedule();
        });



    }


    }
