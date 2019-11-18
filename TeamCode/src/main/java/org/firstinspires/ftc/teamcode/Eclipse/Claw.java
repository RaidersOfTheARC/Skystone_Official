package org.firstinspires.ftc.teamcode.Eclipse;

import com.qualcomm.robotcore.hardware.DcMotor;

public class Claw {

    private DcMotor left, right;

    public Claw(DcMotor l, DcMotor r) {
        left = l;
        right = r;
    }

    public void init() {
        left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        left.setDirection(DcMotor.Direction.FORWARD);
        right.setDirection(DcMotor.Direction.REVERSE);
    }

    public double getSpeed() {
        return (left.getPower() + right.getPower()) / 2;
    }

    public void activate(double pow) {
        left.setPower(pow);
        right.setPower(pow);
    }

}
