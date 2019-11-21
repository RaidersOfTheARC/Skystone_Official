package org.firstinspires.ftc.teamcode.Silver;

import com.qualcomm.robotcore.hardware.Servo;

public class Foundation {

    private Servo left, right;

    public Foundation(Servo l, Servo r) {
        left = l;
        right = r;

        init();
    }

    public void init() {
        left.setDirection(Servo.Direction.FORWARD);
        right.setDirection(Servo.Direction.REVERSE);

        deactivate();
    }

    public boolean isActive() {
        return left.getPosition() != 0 && right.getPosition() != 0;
    }

    public void activate() {
        left.setPosition(1);
        right.setPosition(1);
    }

    public void deactivate() {
        left.setPosition(0);
        right.setPosition(0);
    }

}
