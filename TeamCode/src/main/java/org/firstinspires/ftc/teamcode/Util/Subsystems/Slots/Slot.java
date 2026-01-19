package org.firstinspires.ftc.teamcode.Util.Subsystems.Slots;

import com.bylazar.telemetry.JoinedTelemetry;
import com.qualcomm.robotcore.hardware.ColorSensor;

import org.firstinspires.ftc.teamcode.Util.Subsystems.Robot;
import org.firstinspires.ftc.teamcode.Util.Timer;
import org.firstinspires.ftc.teamcode.Util.UniConstants;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.delays.Delay;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.commands.utility.InstantCommand;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.ftc.ActiveOpMode;
import dev.nextftc.hardware.impl.ServoEx;

public class Slot implements Subsystem {

    private final Timer shotTimer = new Timer();
    private ServoEx light;
    private String lightName;
    private ColorSensor colorSensor;
    private String colorSensorName;
    private ServoEx kickerServo;
    private String kickerName;
    private double up = 0;
    private double down = 0;
    private SlotState colorState = SlotState.EMPTY;
    private ServoState servoState = ServoState.DOWN;
    private JoinedTelemetry telemetry;

    public Slot(String lightName, String colorSensorName, String kickerName, double up, double down) {
        this.up = up;
        this.down = down;
        this.lightName = lightName;
        this.colorSensorName = colorSensorName;
        this.kickerName = kickerName;
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

        //Update Servo
        kickerServo.setPosition((servoState == ServoState.UP) ? up : down);
        light.setPosition(isFull() ? (colorState.equals(SlotState.GREEN) ? .5 : .722) : 0);
    }

    public String getKickerServoName() {
        return UniConstants.LIGHT_LEFT_STRING;
    }

    public Timer getShotTimer() {
        return shotTimer;
    }

    public boolean finishedShot() {
        return shotTimer.getTimeSeconds() < 2;
    }

    public SlotState getColorState() {
        return colorState;
    }

    public ServoState getTargetPosition() {
        return servoState;
    }


    public boolean isFull() {
        return (colorState == SlotState.PURPLE) || (colorState == SlotState.GREEN);
    }


    public Command basicShoot() {
        return new SequentialGroup(
                setServoState(ServoState.UP),
                new Delay(UniConstants.FAST_FLICKER_TIME_UP),
                setServoState(ServoState.DOWN),
                new InstantCommand(shotTimer::reset)
        ).requires("Shooting");
    }

    public Command basicShootDown() {
        return new SequentialGroup(
                setServoState(ServoState.UP),
                new Delay(UniConstants.FAST_FLICKER_TIME_UP),
                setServoState(ServoState.DOWN),
                new Delay(UniConstants.FAST_FLICKER_TIME_DOWN),
                new InstantCommand(shotTimer::reset)
        ).requires("Shooting");
    }

    public Command setServoState(ServoState state) {
        return new InstantCommand(() -> servoState = state);
    }

    public void sendTelemetry(Robot.loggingState state) {
        switch (state) {
            case DISABLED:
                break;
            case ENABLED:
                telemetry.addData("Name ", kickerName);
                telemetry.addData("Color State ", colorState);
                telemetry.addLine();
                break;
            case EXTREME:
                telemetry.addLine("START OF SLOT LOG");
                telemetry.addData("Name ", kickerName);
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

    private void readSlot() {
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

    public enum SlotState {
        PURPLE,
        GREEN,
        EMPTY
    }

    public enum ServoState {
        DOWN,
        UP
    }

}
