package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;

public class StoneElevator {

    private DcMotor lift;       // elevator motor
    private CRServo grabber;    // grabs the nub of the stone

    public StoneElevator(DcMotor mot, CRServo holder) {
        lift = mot;
        grabber = holder;

        init();
    }

    public void init() {
        lift.setDirection(DcMotor.Direction.REVERSE);
        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lift.setPower(0);

        grabber.setDirection(CRServo.Direction.REVERSE);
    }

    public void activate(double pow) {
        lift.setPower(pow);
    }

    public boolean isEngaged() {
        return grabber.getPower() != 0;
    }

    public void engage() {
        if (!isEngaged()) grabber.setPower(1);
    }

    public void disengage() {
        if (isEngaged()) grabber.setPower(0);
    }

    public double getLiftSpeed() {
        return lift.getPower();
    }

}
