package org.firstinspires.ftc.teamcode.Silver;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

public class Intake {

    private DcMotor left, right;    // intake motors
    private Servo holder;           // hold the stone in place

    public Intake(DcMotor left, DcMotor right, Servo holder) {
        this.left = left;
        this.right = right;
        this.holder = holder;

        init();
    }

    public void init() {
        right.setDirection(DcMotor.Direction.REVERSE);
        left.setDirection(DcMotor.Direction.FORWARD);
        holder.setDirection(Servo.Direction.REVERSE);

        left.setPower(0);
        right.setPower(0);
        holder.setPosition(1);
    }

    public void activate(double pow) {
        if (pow > 0) {
            holder.setPosition(0.5);
        } else holder.setPosition(1);

        left.setPower(pow);
        right.setPower(pow);
    }

    public boolean isHeld() {
        return holder.getPosition() == 1;
    }

    public double getIntakeSpeed() {
        return (left.getPower() + right.getPower()) / 2;
    }

}
