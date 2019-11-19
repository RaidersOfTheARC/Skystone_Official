package org.firstinspires.ftc.teamcode.Black;

import com.qualcomm.robotcore.hardware.Servo;

public class Grabber {

    private Servo hold;

    public Grabber(Servo holder) {
        hold = holder;

        init();
    }

    public void init() {
        hold.setDirection(Servo.Direction.FORWARD);

        hold.scaleRange(0.5, 1);
    }

    public boolean isActive() {
        return hold.getPosition() != 0.5;
    }

    public void activate(double pos) {
        hold.setPosition(pos);
    }

}
