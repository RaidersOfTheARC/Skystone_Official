package org.firstinspires.ftc.teamcode.Black;

import com.qualcomm.robotcore.hardware.CRServo;

public class V4BOuttake {

    private CRServo outtake;

    public V4BOuttake(CRServo mec) {
        outtake = mec;

        init();
    }

    public void init() {
        outtake.setDirection(CRServo.Direction.FORWARD);
    }

    public void activate(double pow) {
        outtake.setPower(pow);
    }

    public double getSpeed() {
        return outtake.getPower();
    }

}
