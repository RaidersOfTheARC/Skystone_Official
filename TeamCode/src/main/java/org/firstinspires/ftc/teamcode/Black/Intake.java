package org.firstinspires.ftc.teamcode.Black;

import com.qualcomm.robotcore.hardware.CRServo;

public class Intake {

    private CRServo left, right;

    public Intake(CRServo l, CRServo r) {
        left = l;
        right = r;
        
        init();
    }

    public void init() {
        left.setDirection(CRServo.Direction.FORWARD);
        right.setDirection(CRServo.Direction.REVERSE);
    }

    public void activate(double pow) {
        left.setPower(pow);
        right.setPower(pow);
    }

    public void deactivate() {
        activate(0);
    }

    public double getSpeed() {
        return (left.getPower() + right.getPower()) / 2;
    }

}
