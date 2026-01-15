package org.firstinspires.ftc.teamcode.Util.Subsystems;

import androidx.annotation.NonNull;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.JoinedTelemetry;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Util.Timer;
import org.firstinspires.ftc.teamcode.Util.UniConstants;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashSet;
import java.util.Set;
import java.util.function.Supplier;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.delays.Delay;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.commands.utility.InstantCommand;
import dev.nextftc.core.commands.utility.LambdaCommand;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.ftc.ActiveOpMode;
import dev.nextftc.hardware.impl.MotorEx;
import dev.nextftc.hardware.impl.ServoEx;
import dev.nextftc.hardware.powerable.SetPower;

@Configurable
public class IntakeSortingSubsystem implements Subsystem {
    public static boolean isReversed = false;
    public static servoState state = servoState.DOWN;
    public static Slot backSlot;
    public static Slot rightSlot;
    public static Slot leftSlot;
    public static IntakeSortingSubsystem INSTANCE = new IntakeSortingSubsystem();
    //BANGGGGGGGGGGG
    public static Supplier<ArrayList<Slot>> result;
    public static Timer shotTimer = new Timer();
    public boolean isEnabled = false;
    public ArrayList<Slot> slots;
    //Class variables
    JoinedTelemetry telemetry;
    //Active motor
    MotorEx active = new MotorEx(UniConstants.ACTIVE_INTAKE_STRING).floatMode().reversed();

    public IntakeSortingSubsystem() {
    }

    @Override
    public void initialize() {
        backSlot = new Slot(ActiveOpMode.hardwareMap(), UniConstants.FLICKER_BACK_STRING, UniConstants.COLOR_SENSOR_SLOT_BACK_STRING, UniConstants.LIGHT_BACK_STRING);
        rightSlot = new Slot(ActiveOpMode.hardwareMap(), UniConstants.FLICKER_RIGHT_STRING, UniConstants.COLOR_SENSOR_SLOT_RIGHT_STRING, UniConstants.LIGHT_RIGHT_STRING);
        leftSlot = new Slot(ActiveOpMode.hardwareMap(), UniConstants.FLICKER_LEFT_STRING, UniConstants.COLOR_SENSOR_SLOT_LEFT_STRING, UniConstants.LIGHT_LEFT_STRING);

//        backSlot.setTelemetry(telemetry);
//        rightSlot.setTelemetry(telemetry);
//        leftSlot.setTelemetry(telemetry);

        slots = new ArrayList<>(Arrays.asList(backSlot, rightSlot, leftSlot));
        result = () -> determineOrder(Robot.patternSupplier.get());
    }

    @Override
    public void periodic() {

        if (ActiveOpMode.isStarted()) {

            backSlot.update();
            rightSlot.update();
            leftSlot.update();

            //if ANY servos are up, set state to OUTTAKE so the active is running
            if (backSlot.getTargetPosition().equals(servoState.DOWN) && rightSlot.getTargetPosition().equals(servoState.DOWN) && leftSlot.getTargetPosition().equals(servoState.DOWN)) {
                state = servoState.DOWN;
            } else {
                state = servoState.UP;
            }

            new SetPower(active, isEnabled ? (isReversed ? -1 : 1) : 0).run();


        }

    }

    //Active Methods
    public void disableActive() {
        isEnabled = false;
    }

    public void enableActive() {
        isEnabled = true;
    }

    public void reverseIntake() {
        isReversed = true;
        enableActive();
    }

    public void forwardIntake() {
        isReversed = false;
        enableActive();
    }

    public boolean shouldRumble() {
        return allFull();
    }

    public Command runActive() {
        return new InstantCommand(() -> {
            forwardIntake();
            enableActive();
        });
    }

    public Command stopActive() {
        return new InstantCommand(this::disableActive);
    }

    public ArrayList<Slot> determineOrder(@NonNull ArrayList<Slot.slotState> pattern) {

        ArrayList<Slot> result1 = new ArrayList<>();
        Set<Slot> used = new HashSet<>();
        for (Slot.slotState wanted : pattern) {
            for (Slot slot : slots) {
                if (slot.colorState == wanted && used.add(slot)) {
                    result1.add(slot);
                    break;
                }
            }
        }

        if (result1.size() >= 3) {
            return result1;
        }

        // Not full - add slots that are full first, then empty ones
        ArrayList<Slot> orderedResult = new ArrayList<>();

        // Add full slots first
        for (Slot slot : slots) {
            if (slot.colorState != Slot.slotState.EMPTY) {
                orderedResult.add(slot);
            }
        }

        // Then add empty slots
        for (Slot slot : slots) {
            if (slot.colorState == Slot.slotState.EMPTY) {
                orderedResult.add(slot);
            }
        }

        if (orderedResult.size() < 3) {
            orderedResult = new ArrayList<>(Arrays.asList(backSlot, rightSlot, leftSlot));
        }

        return orderedResult;
    }

    public String slotTele(ArrayList<Slot> awesome) {
        StringBuilder ret = new StringBuilder();
        for (Slot slot : awesome) {
            ret.append(slot.name).append(", ");
        }
        return ret.toString();
    }


    public Command Shoot() {

        Supplier<Slot> first = () -> result.get().get(0);
        Supplier<Slot> second = () -> result.get().get(1);
        Supplier<Slot> third = () -> result.get().get(2);


        return new SequentialGroup(
                new LambdaCommand()
                        .setStart(() -> {
                            backSlot.readSlot();
                            rightSlot.readSlot();
                            leftSlot.readSlot();
                        }),
                new SequentialGroup(
                        new LambdaCommand()
                                .setStart(() -> first.get().basicShootDown().named("Result 1 " + first.get().getName()).run())
                                .setIsDone(() -> first.get().finishedShot()),
                        new LambdaCommand()
                                .setStart(() -> second.get().basicShootDown().named("Result 2 " + second.get().getName()).run())
                                .setIsDone(() -> second.get().finishedShot()),
                        new LambdaCommand()
                                .setStart(() -> third.get().basicShoot().named("Result 3 " + third.get().getName()).run())
                                .setIsDone(() -> third.get().finishedShot())
                ),
                new InstantCommand(() -> shotTimer.reset())
        ).setInterruptible(false).addRequirements(backSlot, rightSlot, leftSlot);
    }


    public boolean allFull() {
        return backSlot.isFull() && rightSlot.isFull() && leftSlot.isFull();
    }

    public boolean allEmpty() {
        return !backSlot.isFull() && !rightSlot.isFull() && !leftSlot.isFull();
    }


    public void sendSlotTelemetry(Robot.loggingState state) {
        backSlot.sendTelemetry(state);
        rightSlot.sendTelemetry(state);
        leftSlot.sendTelemetry(state);
    }

    public void sendTelemetry(Robot.loggingState state) {
        switch (state) {
            case DISABLED:
                break;
            case ENABLED:
                telemetry.addLine("START OF SORTING LOG");
                telemetry.addData("Result: ", slotTele(result.get()));
                telemetry.addData("Intake Enabled ", isEnabled);
                telemetry.addData("Intake Reversed ", isReversed);
                telemetry.addData("All Full ", allFull());
                sendSlotTelemetry(state);
                telemetry.addLine("END OF SORTING LOG");
                telemetry.addLine();
                break;
            case EXTREME:
                telemetry.addLine("START OF ROTARY LOG");
                telemetry.addLine("END OF ROTARY LOG");


        }
    }

    public void setTelemetry(JoinedTelemetry telemetry) {
        this.telemetry = telemetry;
    }

    public enum servoState {
        DOWN,
        UP
    }


    public static class Slot {

        private final ServoEx kickerServo;
        private final ServoEx light;
        private final ColorSensor colorSensor;
        private final Timer shotTimer = new Timer();
        String name = "";
        double up = 0;
        double down = 0;
        private slotState colorState = slotState.EMPTY;
        private servoState servoState = IntakeSortingSubsystem.servoState.DOWN;
        private JoinedTelemetry telemetry;

        public Slot(HardwareMap hardwareMap, String kickerServoName, String colorSensorName, String lightName) {

            kickerServo = new ServoEx(kickerServoName);
            light = new ServoEx(lightName);
            name = kickerServoName;
            colorSensor = hardwareMap.get(ColorSensor.class, colorSensorName);

            //Identify slot and assign servo constants
            if (name.equals(UniConstants.FLICKER_BACK_STRING)) {
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


        public void update() {

//            //ONLY UNCOMMENT FOR TESTING
//            if(name.equals(UniConstants.FLICKER_BACK_STRING)){
//                up = UniConstants.FLICKER_BACK_UP; //Back up
//                down = UniConstants.FLICKER_BACK_DOWN; //Back down
//            } else if (name.equals(UniConstants.FLICKER_LEFT_STRING)) {
//                up = UniConstants.FLICKER_LEFT_UP; //Left up
//                down = UniConstants.FLICKER_LEFT_DOWN; //Left down
//            } else {
//                up = UniConstants.FLICKER_RIGHT_UP; //Right up
//                down = UniConstants.FLICKER_RIGHT_DOWN; //Right down
//            }

            //Update Colors
            readSlot();

            //Update Servo
            kickerServo.setPosition(state == IntakeSortingSubsystem.servoState.UP ? ((servoState == IntakeSortingSubsystem.servoState.UP) ? up : down) : down);
            light.setPosition(isFull() ? (colorState.equals(slotState.GREEN) ? .5 : .722) : 0);
        }

        public String getName() {
            return name;
        }

        public Timer getShotTimer() {
            return shotTimer;
        }

        public boolean finishedShot() {
            return shotTimer.getTimeSeconds() < 2;
        }

        private void readSlot() {
            double red = colorSensor.red();
            double green = colorSensor.green();
            double blue = colorSensor.blue();
            double alpha = colorSensor.alpha();
            if (((green > red) && (blue > red)) && (alpha < 5000) && green > 65) {
                colorState = slotState.GREEN;
            } else if (((red > green) && (blue > green)) && (alpha < 5000)) {
                colorState = slotState.PURPLE;
            } else if (IntakeSortingSubsystem.shotTimer.getTimeSeconds() < 2) {
                colorState = slotState.EMPTY;
            }
        }

        public slotState getColorState() {
            return colorState;
        }

        public servoState getTargetPosition() {
            return servoState;
        }

        public void setTargetPosition(servoState state) {
            servoState = state;
        }

        public boolean isFull() {
            return (colorState == slotState.PURPLE) || (colorState == slotState.GREEN);
        }


        public Command basicShoot() {
            return new SequentialGroup(
                    setServoState(IntakeSortingSubsystem.servoState.UP),
                    new Delay(UniConstants.FAST_FLICKER_TIME_UP),
                    setServoState(IntakeSortingSubsystem.servoState.DOWN),
                    new InstantCommand(shotTimer::reset)
            ).requires("Shooting");
        }

        public Command basicShootDown() {
            return new SequentialGroup(
                    setServoState(IntakeSortingSubsystem.servoState.UP),
                    new Delay(UniConstants.FAST_FLICKER_TIME_UP),
                    setServoState(IntakeSortingSubsystem.servoState.DOWN),
                    new Delay(UniConstants.FAST_FLICKER_TIME_DOWN),
                    new InstantCommand(shotTimer::reset)
            ).requires("Shooting");
        }

        public Command setServoState(servoState state) {
            return new InstantCommand(() -> servoState = state);
        }

        public void sendTelemetry(Robot.loggingState state) {
            switch (state) {
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


        public void setTelemetry(JoinedTelemetry telemetry) {
            this.telemetry = telemetry;
        }


        public enum slotState {
            PURPLE,
            GREEN,
            EMPTY
        }
    }
}