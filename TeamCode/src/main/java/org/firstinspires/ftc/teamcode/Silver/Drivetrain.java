package org.firstinspires.ftc.teamcode.Silver;

import com.qualcomm.robotcore.hardware.DcMotor;

public class Drivetrain {

    private DcMotor fL, fR, bL, bR;

    public Drivetrain(DcMotor lf, DcMotor rf, DcMotor lb, DcMotor rb) {
        fL = lf;
        fR = rf;
        bL = lb;
        bR = rb;

        init();
    }

    public void init() {
        fL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        fR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        fL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        fL.setDirection(DcMotor.Direction.FORWARD);
        fR.setDirection(DcMotor.Direction.REVERSE);
        bL.setDirection(DcMotor.Direction.FORWARD);
        bR.setDirection(DcMotor.Direction.REVERSE);
    }

    public void drive(double x, double y, double turn, double mult) {
        double[] speeds = driveMecanum(x, y, turn);
        fL.setPower(mult * speeds[0]);
        fR.setPower(mult * speeds[1]);
        bL.setPower(mult * speeds[2]);
        bR.setPower(mult * speeds[3]);
    }

    public double[] getSpeeds() {
        return new double[]{fL.getPower(), fR.getPower(), bL.getPower(), bR.getPower()};
    }

    private double[] driveMecanum(double x, double y, double turn) {
        double[] speeds = new double[4];

        double j = Math.hypot(x,y);
        double theta = Math.atan2(-y,x);

        speeds[0] = j * Math.sin(theta + Math.PI / 4) + turn;  // fL
        speeds[1] = j * Math.sin(theta - Math.PI / 4) - turn;  // fR
        speeds[2] = j * Math.sin(theta - Math.PI / 4) + turn;  // bL
        speeds[3] = j * Math.sin(theta + Math.PI / 4) - turn;  // bR

        return speeds;
    }
    
    public DcMotor[] getMotors() {
        return new DcMotor[]{fL, fR, bL, bR};
    }

}
