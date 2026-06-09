package org.firstinspires.ftc.teamcode.Subsystems.Slots;

import com.qualcomm.robotcore.hardware.ColorSensor;

import org.firstinspires.ftc.teamcode.Subsystems.RobotSubsystem;
import org.firstinspires.ftc.teamcode.Util.Timer;
import org.firstinspires.ftc.teamcode.Util.UniConstants;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.delays.Delay;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.commands.utility.InstantCommand;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.ftc.ActiveOpMode;
import dev.nextftc.hardware.impl.ServoEx;

public class MainSlot implements Subsystem {

    private final Timer shotTimer = new Timer();
    private ServoEx light;
    private final String lightName;
    private ColorSensor colorSensor;
    private final String colorSensorName;
    private ServoEx kickerServo;
    private final String kickerName;
    private double up = 0;
    private double down = 0;
    private double jigglePos = 0;
    private SlotState colorState = SlotState.EMPTY;
    private ServoState servoState = ServoState.DOWN;
    private final double oldUp = 0;
    private final double oldDown = 0;

    public MainSlot(String lightName, String colorSensorName, String kickerName, double up, double down, double jigglePos) {
        this.lightName = lightName;
        this.colorSensorName = colorSensorName;
        this.kickerName = kickerName;
        this.up = up;
        this.down = down;
        this.jigglePos = jigglePos;
    }


    @Override
    public void initialize() {
        kickerServo = new ServoEx(kickerName);
        light = new ServoEx(lightName);
        colorSensor = ActiveOpMode.hardwareMap().get(ColorSensor.class, colorSensorName);

    }

    @Override
    public void periodic() {
        //Update Colors
        readSlot();

        switch (kickerName) {

            case UniConstants.FLICKER_LEFT_STRING:
                setValues(UniConstants.FLICKER_LEFT_UP, UniConstants.FLICKER_LEFT_DOWN);
                break;
            case UniConstants.FLICKER_BACK_STRING:
                setValues(UniConstants.FLICKER_BACK_UP, UniConstants.FLICKER_BACK_DOWN);
                break;
            case UniConstants.FLICKER_RIGHT_STRING:
                setValues(UniConstants.FLICKER_RIGHT_UP, UniConstants.FLICKER_RIGHT_DOWN);
                break;


        }

        //Update Servo
        kickerServo.setPosition((servoState == ServoState.UP) ? up : servoState == ServoState.DOWN ? down : jigglePos);
        light.setPosition(isFull() ? (colorState.equals(SlotState.GREEN) ? .5 : .722) : 0);
    }

    public String getKickerServoName() {
        return kickerName;
    }

    public SlotState getColorState() {
        return colorState;
    }


    public Command basicShoot() {
        return new SequentialGroup(
                setServoState(ServoState.UP),
                new Delay(UniConstants.FAST_FLICKER_TIME_UP),
                setServoState(ServoState.DOWN)

        ).requires("Shooting");
    }

    public Command basicShootDown() {
        return new SequentialGroup(
                setServoState(ServoState.UP),
                new Delay(UniConstants.FAST_FLICKER_TIME_UP),
                setServoState(ServoState.DOWN),
                new Delay(UniConstants.FAST_FLICKER_TIME_DOWN + RobotSubsystem.INSTANCE.getShootDelay())
        ).requires("Shooting");
    }

    public Command setServoState(ServoState state) {
        return new InstantCommand(() -> servoState = state);
    }


    public boolean isFull() {
        return (colorState == SlotState.PURPLE) || (colorState == SlotState.GREEN);
    }

    public void readSlot() {
        double red = colorSensor.red();
        double green = colorSensor.green();
        double blue = colorSensor.blue();
        double alpha = colorSensor.alpha();


        if (((green > red) && (blue > red)) && (alpha < 5000) && green > 65) {
            colorState = SlotState.GREEN;
        } else if (((red > green) && (blue > green)) && (alpha < 5000)) {
            colorState = SlotState.PURPLE;
        } else {
            colorState = SlotState.EMPTY;
        }
    }

    public Command jiggle() {
        return new SequentialGroup(
                setServoState(ServoState.JIGGLE),
                new Delay(.05),
                setServoState(ServoState.DOWN)
        );
    }

    public void setValues(double up, double down) {
        this.up = up;
        this.down = down;
    }


    public enum SlotState {
        PURPLE,
        GREEN,
        EMPTY
    }

    public enum ServoState {
        DOWN,
        UP,
        JIGGLE
    }
}
