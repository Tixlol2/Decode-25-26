package org.firstinspires.ftc.teamcode.OpModes.Paths;

import org.firstinspires.ftc.teamcode.Util.Poses;

public class Close9 extends MainPaths {

    public Close9() {
        super("Close 9", 4, ShootingLocation.SHORT, Poses.blueGoalTopStartFacing);
        init();
    }

    public void init() {
        setCommand(1, MoveShoot());
        setCommand(2, IntakeTop());
        setCommand(3, MoveShoot());
        setCommand(4, IntakeMid());
        setCommand(5, MoveShoot());
        setCommand(6, Park());
    }


}
