package org.firstinspires.ftc.teamcode.Util.Subsystems;

import androidx.annotation.NonNull;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.JoinedTelemetry;
import com.bylazar.telemetry.PanelsTelemetry;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Util.UniConstants;

import java.util.ArrayList;
import java.util.Arrays;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.delays.Delay;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.commands.utility.InstantCommand;
import dev.nextftc.core.commands.utility.LambdaCommand;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.ftc.ActiveOpMode;
import dev.nextftc.hardware.impl.MotorEx;
import dev.nextftc.hardware.impl.ServoEx;

@Configurable
public class IntakeSortingSubsystem implements Subsystem {
    //Class variables
    JoinedTelemetry telemetry;
    public static boolean debug = false;
    public boolean isEnabled = false;
    public static boolean isReversed = false;

    public static final IntakeSortingSubsystem INSTANCE = new IntakeSortingSubsystem();

    //Active motor
    MotorEx active = new MotorEx(UniConstants.ACTIVE_INTAKE_STRING).floatMode().reversed();
    public static UniConstants.servoState state = UniConstants.servoState.DOWN;

    public ArrayList<UniConstants.slotState> shotHistory = new ArrayList<>();

    public static Slot backSlot;
    public static Slot rightSlot;
    public static Slot leftSlot;

    public ArrayList<Slot> slots;

    public IntakeSortingSubsystem(){}

    @Override
    public void initialize(){
        telemetry = new JoinedTelemetry(ActiveOpMode.telemetry(), PanelsTelemetry.INSTANCE.getFtcTelemetry());

        backSlot = new Slot(ActiveOpMode.hardwareMap(), UniConstants.FLICKER_BACK_STRING, UniConstants.COLOR_SENSOR_SLOT_BACK_STRING, UniConstants.LIGHT_BACK_STRING, telemetry);
        rightSlot = new Slot(ActiveOpMode.hardwareMap(), UniConstants.FLICKER_RIGHT_STRING, UniConstants.COLOR_SENSOR_SLOT_RIGHT_STRING, UniConstants.LIGHT_RIGHT_STRING, telemetry);
        leftSlot = new Slot(ActiveOpMode.hardwareMap(), UniConstants.FLICKER_LEFT_STRING, UniConstants.COLOR_SENSOR_SLOT_LEFT_STRING, UniConstants.LIGHT_LEFT_STRING, telemetry);

        slots = new ArrayList<>(Arrays.asList(backSlot, rightSlot, leftSlot));


    }


    @Override
    public void periodic() {

        if(ActiveOpMode.isStarted()) {

            backSlot.update();
            rightSlot.update();
            leftSlot.update();

            //if ANY servos are up, set state to OUTTAKE so the active is running
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
        enableActive();
    }

    public void forwardIntake(){
        isReversed = false;
        enableActive();
    }
    public boolean shouldRumble(){return allFull();}

    public Command runActive(){
        return new InstantCommand(() -> {forwardIntake(); enableActive();});
    }
    public Command stopActive(){
        return new InstantCommand(this::disableActive);
    }

    public Command setServoState(Slot slot, UniConstants.servoState state) {
        return new InstantCommand(() -> {slot.setTargetPosition(state);}).named(slot.name + " Set Servo State");
    }

    /**
     * Two-phase adaptive shooting: Launch current balls, then rescan and launch any that shifted in
     * Uses proper command scheduling - NO blocking calls
     * Tracks shot history in shotHistory ArrayList
     *
     * Replace your existing shoot() method with this complete implementation
     */
    public Command shoot(@NonNull ArrayList<UniConstants.slotState> pattern){
        return new SequentialGroup(
                // Phase 1: Launch all currently detected balls
                createInitialLaunchCommand(pattern),

                // Small delay to let mechanical system settle
                new Delay(UniConstants.FAST_FLICKER_TIME_DOWN),

                // Phase 2: Rescan and launch any new balls that appeared
                createRescanLaunchCommand()
        ).named("Adaptive Shoot");
    }

    /**
     * Phase 1: Determine and launch initial balls
     */
    private Command createInitialLaunchCommand(ArrayList<UniConstants.slotState> pattern){
        return new LambdaCommand()
                .setStart(() -> {
                    // Read current state
                    UniConstants.slotState s0 = slots.get(0).getColorState();
                    UniConstants.slotState s1 = slots.get(1).getColorState();
                    UniConstants.slotState s2 = slots.get(2).getColorState();

                    boolean s0Full = s0 != UniConstants.slotState.EMPTY;
                    boolean s1Full = s1 != UniConstants.slotState.EMPTY;
                    boolean s2Full = s2 != UniConstants.slotState.EMPTY;

                    ArrayList<Slot> toLaunch = new ArrayList<>(3);
                    boolean[] used = new boolean[3];

                    // Try to match pattern
                    if(pattern.size() >= 3){
                        UniConstants.slotState p0 = pattern.get(0);
                        UniConstants.slotState p1 = pattern.get(1);
                        UniConstants.slotState p2 = pattern.get(2);

                        // Check if we can make exact pattern
                        int matches = 0;
                        boolean[] temp = new boolean[3];

                        if(s0.equals(p0)){ temp[0] = true; matches++; }
                        else if(s1.equals(p0)){ temp[1] = true; matches++; }
                        else if(s2.equals(p0)){ temp[2] = true; matches++; }

                        if(!temp[0] && s0.equals(p1)){ temp[0] = true; matches++; }
                        else if(!temp[1] && s1.equals(p1)){ temp[1] = true; matches++; }
                        else if(!temp[2] && s2.equals(p1)){ temp[2] = true; matches++; }

                        if(!temp[0] && s0.equals(p2)){ temp[0] = true; matches++; }
                        else if(!temp[1] && s1.equals(p2)){ temp[1] = true; matches++; }
                        else if(!temp[2] && s2.equals(p2)){ temp[2] = true; matches++; }

                        if(matches == 3){
                            // Build exact pattern
                            if(s0.equals(p0)){ toLaunch.add(backSlot); used[0] = true; }
                            else if(s1.equals(p0)){ toLaunch.add(rightSlot); used[1] = true; }
                            else if(s2.equals(p0)){ toLaunch.add(leftSlot); used[2] = true; }

                            if(!used[0] && s0.equals(p1)){ toLaunch.add(backSlot); used[0] = true; }
                            else if(!used[1] && s1.equals(p1)){ toLaunch.add(rightSlot); used[1] = true; }
                            else if(!used[2] && s2.equals(p1)){ toLaunch.add(leftSlot); used[2] = true; }

                            if(!used[0] && s0.equals(p2)){ toLaunch.add(backSlot); used[0] = true; }
                            else if(!used[1] && s1.equals(p2)){ toLaunch.add(rightSlot); used[1] = true; }
                            else if(!used[2] && s2.equals(p2)){ toLaunch.add(leftSlot); used[2] = true; }
                        }
                    }

                    // If pattern failed or not enough matches, just launch all full slots
                    if(toLaunch.isEmpty()){
                        // Prioritize right/left over back for first position
                        boolean onlyBackFull = s0Full && !s1Full && !s2Full;

                        if(onlyBackFull){
                            toLaunch.add(backSlot);
                            used[0] = true;
                        } else {
                            if(s1Full){ toLaunch.add(rightSlot); used[1] = true; }
                            else if(s2Full){ toLaunch.add(leftSlot); used[2] = true; }
                        }

                        if(!used[0] && s0Full) toLaunch.add(backSlot);
                        if(!used[1] && s1Full) toLaunch.add(rightSlot);
                        if(!used[2] && s2Full) toLaunch.add(leftSlot);
                    }

                    // Schedule the batch launch
                    if(!toLaunch.isEmpty()){
                        batchLaunch(toLaunch.toArray(new Slot[0])).schedule();
                    }
                })
                .setIsDone(() -> true)
                .named("Initial Launch");
    }

    /**
     * Phase 2: Rescan for any balls that shifted in during phase 1
     */
    private Command createRescanLaunchCommand(){
        return new LambdaCommand()
                .setStart(() -> {
                    // Rescan all slots
                    ArrayList<Slot> remainingBalls = new ArrayList<>();

                    if(slots.get(0).isFull()) remainingBalls.add(backSlot);
                    if(slots.get(1).isFull()) remainingBalls.add(rightSlot);
                    if(slots.get(2).isFull()) remainingBalls.add(leftSlot);

                    // Launch any balls found
                    if(!remainingBalls.isEmpty()){
                        batchLaunch(remainingBalls.toArray(new Slot[0])).schedule();
                    }
                })
                .setIsDone(() -> true)
                .named("Rescan Launch");
    }

    /**
     * Launches multiple slots in sequence with real-time verification
     * Records each successful launch to shotHistory
     */
    private Command batchLaunch(Slot[] slotsToLaunch){
        if(slotsToLaunch.length == 0){
            return new InstantCommand(() -> {});
        }

        // Build dynamic command sequence
        ArrayList<Command> commands = new ArrayList<>();

        for(int i = 0; i < slotsToLaunch.length; i++){
            final Slot slot = slotsToLaunch[i];

            // Verify and launch, record to history if successful
            commands.add(new LambdaCommand()
                    .setStart(() -> {
                        if(slot.isFull()){
                            UniConstants.slotState colorShot = slot.getColorState();
                            slot.setTargetPosition(UniConstants.servoState.UP);
                            // Add to shot history
                            shotHistory.add(colorShot);
                        }
                    })
                    .setIsDone(() -> true)
                    .named(slot.name + " Up"));

            commands.add(new Delay(UniConstants.FAST_FLICKER_TIME_UP));

            commands.add(new InstantCommand(() ->
                    slot.setTargetPosition(UniConstants.servoState.DOWN))
                    .named(slot.name + " Down"));

            // Add delay between shots (but not after last shot)
            if(i < slotsToLaunch.length - 1){
                commands.add(new Delay(UniConstants.FAST_FLICKER_TIME_DOWN));
            }
        }

        return new SequentialGroup(commands.toArray(new Command[0]));
    }


    public void clearShotHistory(){
        shotHistory.clear();
    }


    public int getPurpleCount(){
        int count = 0;
        for(UniConstants.slotState color : shotHistory){
            if(color == UniConstants.slotState.PURPLE) count++;
        }
        return count;
    }

    public int getGreenCount(){
        int count = 0;
        for(UniConstants.slotState color : shotHistory){
            if(color == UniConstants.slotState.GREEN) count++;
        }
        return count;
    }






    public boolean allFull() {
        return backSlot.isFull() && rightSlot.isFull() && leftSlot.isFull();
    }

    public boolean isFast(Slot slot){
        return slot.name.equals("FLS") || slot.name.equals("FBS");
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
                telemetry.addData("Shot History ", shotHistory);
                telemetry.addData("Purple Count ", getPurpleCount());
                telemetry.addData("Green Count ", getGreenCount());
                telemetry.addLine("END OF SORTING LOG");
                telemetry.addLine();
                break;
            case EXTREME:
                telemetry.addLine("START OF ROTARY LOG");
                telemetry.addLine("END OF ROTARY LOG");


        }
    }


    public static class Slot {

        private final ServoEx kickerServo;
        private final ServoEx light;
        private final ColorSensor colorSensor;

        String name = "";

        private UniConstants.slotState colorState = UniConstants.slotState.EMPTY;
        private UniConstants.servoState servoState = UniConstants.servoState.DOWN;
        private final JoinedTelemetry telemetry;

        double up = 0;
        double down = 0;

        public Slot(HardwareMap hardwareMap, String kickerServoName, String colorSensorName, String lightName, JoinedTelemetry telemetry){

            kickerServo = new ServoEx(kickerServoName);
            light = new ServoEx(lightName);
            name = kickerServoName;
            colorSensor = hardwareMap.get(ColorSensor.class, colorSensorName);

            this.telemetry = telemetry;

            //Identify slot and assign servo constants
            if(name.equals(UniConstants.FLICKER_BACK_STRING)){
                up = UniConstants.FLICKER_BACK_UP; //Back up
                down = UniConstants.FLICKER_BACK_DOWN; //Back down
            } else if (name.equals(UniConstants.FLICKER_LEFT_STRING)) {
                up = UniConstants.FLICKER_LEFT_UP; //Left up
                down = UniConstants.FLICKER_LEFT_DOWN; //Left down
            } else {
                up = UniConstants.FLICKER_RIGHT_UP; //Right up
                down = UniConstants.FLICKER_RIGHT_DOWN; //Right down
            }



        }

        public void update(){

            //ONLY UNCOMMENT FOR TESTING
            if(name.equals(UniConstants.FLICKER_BACK_STRING)){
                up = UniConstants.FLICKER_BACK_UP; //Back up
                down = UniConstants.FLICKER_BACK_DOWN; //Back down
            } else if (name.equals(UniConstants.FLICKER_LEFT_STRING)) {
                up = UniConstants.FLICKER_LEFT_UP; //Left up
                down = UniConstants.FLICKER_LEFT_DOWN; //Left down
            } else {
                up = UniConstants.FLICKER_RIGHT_UP; //Right up
                down = UniConstants.FLICKER_RIGHT_DOWN; //Right down
            }


            //Update Colors
            readSlot();

            //Update Servo
            kickerServo.setPosition(state == UniConstants.servoState.UP ? ((servoState == UniConstants.servoState.UP) ? up : down) : down);
            light.setPosition(isFull() ? (colorState.equals(UniConstants.slotState.GREEN) ? .5 : .722) : 0);
        }

        private void readSlot() {

            double red = colorSensor.red();
            double green = colorSensor.green();
            double blue = colorSensor.blue();
            double alpha = colorSensor.alpha();
            if (((green > red) && (blue > red)) && (alpha < 5000) && green > 55) {
                colorState = UniConstants.slotState.GREEN;
            } else if (((red > green) && (blue > green)) && (alpha < 5000)) {
                colorState = UniConstants.slotState.PURPLE;
            } else {
                colorState = UniConstants.slotState.EMPTY;
            }


        }

        public UniConstants.slotState getColorState(){
            return colorState;
        }

        public UniConstants.servoState getTargetPosition(){
            return servoState;
        }

        public void setTargetPosition(UniConstants.servoState state){
            servoState = state;
        }

        public boolean isFull() {
            return (colorState == UniConstants.slotState.PURPLE) || (colorState == UniConstants.slotState.GREEN);
        }

        public void sendTelemetry(UniConstants.loggingState state){
            switch(state){
                case DISABLED:
                    break;
                case ENABLED:
                    telemetry.addData("Name ", name);
                    telemetry.addData("Color State ", colorState);
                    telemetry.addLine();
                    break;
                case EXTREME:
                    telemetry.addLine("START OF SLOT LOG");
                    telemetry.addData("Name ", name);
                    telemetry.addData("Kicker Up ", up);
                    telemetry.addData("Kicker Down ", down);
                    telemetry.addData("Color State ", colorState);
                    telemetry.addData("Red ", colorSensor.red());
                    telemetry.addData("Green ", colorSensor.green());
                    telemetry.addData("Blue ", colorSensor.blue());
                    telemetry.addLine();
                    telemetry.addData("Is Full ", isFull());
                    telemetry.addLine("END OF SLOT LOG");
                    telemetry.addLine();
                    break;

            }







        }



    }
}
