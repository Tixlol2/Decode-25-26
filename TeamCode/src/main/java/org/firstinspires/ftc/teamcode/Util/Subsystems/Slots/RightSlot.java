package org.firstinspires.ftc.teamcode.Util.Subsystems.Slots;

import org.firstinspires.ftc.teamcode.Util.UniConstants;

public class RightSlot extends Slot {

    public static final RightSlot INSTANCE = new RightSlot();

    public RightSlot() {
        super(UniConstants.LIGHT_RIGHT_STRING, UniConstants.COLOR_SENSOR_SLOT_RIGHT_STRING, UniConstants.FLICKER_RIGHT_STRING, UniConstants.FLICKER_RIGHT_UP, UniConstants.FLICKER_RIGHT_DOWN);
    }


}