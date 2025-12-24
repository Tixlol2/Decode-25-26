package org.firstinspires.ftc.teamcode.Util.Subsystems;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.JoinedTelemetry;
import com.bylazar.telemetry.PanelsTelemetry;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Util.UniConstants;

import java.lang.reflect.Array;
import java.util.ArrayList;
import java.util.Arrays;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.delays.Delay;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.commands.utility.LambdaCommand;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.ftc.ActiveOpMode;
import dev.nextftc.hardware.impl.MotorEx;

@Configurable
public class IntakeSortingSubsystem implements Subsystem {
    //Class variables
    JoinedTelemetry telemetry;
    public static boolean debug = false;
    public static boolean isEnabled = false;
    public static boolean isReversed = false;






    //Active motor
    MotorEx active = new MotorEx(UniConstants.ACTIVE_INTAKE_STRING).floatMode().reversed();
    public static UniConstants.servoState state = UniConstants.servoState.DOWN;

    public static Slot backSlot;
    public static Slot rightSlot;
    public static Slot leftSlot;

    public ArrayList<Slot> slots;

    public IntakeSortingSubsystem(){}

    @Override
    public void initialize(){
        telemetry = new JoinedTelemetry(ActiveOpMode.telemetry(), PanelsTelemetry.INSTANCE.getFtcTelemetry());

        backSlot = new Slot(ActiveOpMode.hardwareMap(), UniConstants.FLICKER_BACK_STRING, UniConstants.COLOR_SENSOR_SLOT_BACK_STRING, telemetry);
        rightSlot = new Slot(ActiveOpMode.hardwareMap(), UniConstants.FLICKER_RIGHT_STRING, UniConstants.COLOR_SENSOR_SLOT_RIGHT_STRING, telemetry);
        leftSlot = new Slot(ActiveOpMode.hardwareMap(), UniConstants.FLICKER_LEFT_STRING, UniConstants.COLOR_SENSOR_SLOT_LEFT_STRING, telemetry);

        slots = new ArrayList<>(Arrays.asList(backSlot, rightSlot, leftSlot));
    }

    /*
    * Back slot down = .2
    * Back slot up = .5
    *
    *
    *
    *
    *
    *
    * */


    @Override
    public void periodic() {

        if(ActiveOpMode.isStarted()) {

            backSlot.update();
            rightSlot.update();
            leftSlot.update();

            //if ANY servos are up, set state to OUTTAKE so the active is running
            //TODO: Make sure the flicker up and down are the right values for all servos
            if (backSlot.getTargetPosition().equals(UniConstants.servoState.DOWN) && rightSlot.getTargetPosition().equals(UniConstants.servoState.DOWN) && leftSlot.getTargetPosition().equals(UniConstants.servoState.DOWN)) {
                state = UniConstants.servoState.DOWN;
            } else {
                state = UniConstants.servoState.UP;
            }


            active.setPower(isEnabled ? (isReversed ? -1 : 1) : 0);

        }

    }

    //Active Methods
    public void disableActive(){
        isEnabled = false;
    }

    public void enableActive(){
        isEnabled = true;
    }

    public void reverseIntake(){
        isReversed = true;
    }

    public void forwardIntake(){
        isReversed = false;
    }
    public boolean shouldRumble(){return state == UniConstants.servoState.UP;}

    //Command Testing
    public Command setServoState(Slot slot, UniConstants.servoState state){
        return new LambdaCommand()
                .setStart(() -> {
                    // Runs on start
                    slot.setTargetPosition(state);

                })
                .setUpdate(() -> {
                    // Runs on update
                })
                .setStop(interrupted -> {
                    // Runs on stop
                    slot.setTargetPosition(UniConstants.servoState.DOWN);
                })
                .setIsDone(() -> true) // Returns if the command has finished
                .requires(this)
                .setInterruptible(false)
                .named("Set Servo State"); // sets the name of the command; optional
    }

    public Command launchInPattern(Slot first, Slot second, Slot third){
        return new SequentialGroup(
                setServoState(first, UniConstants.servoState.UP),
                new Delay(1),
                setServoState(first, UniConstants.servoState.DOWN),
                new Delay(1),
                setServoState(second, UniConstants.servoState.UP),
                new Delay(1),
                setServoState(second, UniConstants.servoState.DOWN),
                new Delay(1),
                setServoState(third, UniConstants.servoState.UP),
                new Delay(1),
                setServoState(third, UniConstants.servoState.DOWN)
        );
    }

    public Command shoot(ArrayList<UniConstants.slotState> pattern){

        int loops = 0;

        Slot first = backSlot;
        Slot second = rightSlot;
        Slot third = leftSlot;

        ArrayList<Slot> used = new ArrayList<>();

        //TODO: research how to make this better :sob:
        for(UniConstants.slotState state : pattern){
            loops++;
            for(Slot slot : slots){
                if(state == slot.getColorState() && !used.contains(slot)){
                    switch(loops){
                        case 1:
                            first = slot;
                            break;
                        case 2:
                            second = slot;
                            break;
                        case 3:
                            third = slot;
                            break;
                        default:
                            break;
                    }
                    used.add(slot);
                    break;
                }

            }
        }

        return launchInPattern(first, second, third);
    }

    public boolean allFull() {
        return backSlot.isFull() && rightSlot.isFull() && leftSlot.isFull();
    }


    public void sendTelemetry(UniConstants.loggingState state){
        switch(state){
            case DISABLED:
                break;
            case ENABLED:
                telemetry.addLine("START OF SORTING LOG");
                telemetry.addData("Intake Enabled ", isEnabled);
                telemetry.addData("Intake Reversed ", isReversed);
                telemetry.addData("All Full ", allFull());
                telemetry.addLine("END OF SORTING LOG");
                telemetry.addLine();
                break;
            case EXTREME:
                telemetry.addLine("START OF ROTARY LOG");
                telemetry.addLine("END OF ROTARY LOG");


        }
    }




}
