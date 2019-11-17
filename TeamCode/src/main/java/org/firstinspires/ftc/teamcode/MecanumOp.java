package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="Drive Testing")
// @Disabled
public class MecanumOp extends OpMode {

    private DcMotor fL, fR, bL, bR, lift, intakeL, intakeR;
    private Servo foundationL, foundationR, blockHolder;
    private CRServo grabber, rackL, rackR;

    private double driveMag = 0.5;
    private boolean blockIsHeld = false;

    @Override
    public void init() {
        fL = hardwareMap.get(DcMotor.class, "leftFront");
        fR = hardwareMap.get(DcMotor.class, "rightFront");
        bL = hardwareMap.get(DcMotor.class, "leftBack");
        bR = hardwareMap.get(DcMotor.class, "rightBack");
        lift = hardwareMap.get(DcMotor.class, "lift");
        intakeL = hardwareMap.get(DcMotor.class, "intakeLeft");
        intakeR = hardwareMap.get(DcMotor.class, "intakeRight");

        foundationL = hardwareMap.get(Servo.class, "foundationL");
        foundationR = hardwareMap.get(Servo.class, "foundationR");
        grabber = hardwareMap.get(CRServo.class, "grabber");
        rackL = hardwareMap.get(CRServo.class, "forward1");
        rackR = hardwareMap.get(CRServo.class, "forward2");
        blockHolder = hardwareMap.get(Servo.class, "blockHolder");

        fL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        fR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        fL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        fL.setDirection(DcMotor.Direction.FORWARD);
        fR.setDirection(DcMotor.Direction.REVERSE);
        bL.setDirection(DcMotor.Direction.FORWARD);
        bR.setDirection(DcMotor.Direction.REVERSE);
        lift.setDirection(DcMotor.Direction.REVERSE);
        intakeR.setDirection(DcMotor.Direction.REVERSE);
        intakeL.setDirection(DcMotor.Direction.FORWARD);

        foundationL.setDirection(Servo.Direction.FORWARD);
        foundationR.setDirection(Servo.Direction.REVERSE);
        blockHolder.setDirection(Servo.Direction.REVERSE);
        grabber.setDirection(CRServo.Direction.REVERSE);
        // rackL.setDirection(CRServo.Direction.FORWARD);
        // rackR.setDirection(CRServo.Direction.REVERSE);

        foundationL.scaleRange(0, 0.8);
        foundationR.scaleRange(0, 0.8);
        foundationL.setPosition(0);
        foundationR.setPosition(0);
        blockHolder.setPosition(1);
        grabber.setPower(0);
    }

    @Override
    public void loop() {
        double speeds[] = driveMecanum(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x);

        telemetry.addData("Drive Speed Multiplier", driveMag);

        fL.setPower(driveMag * speeds[0]);
        fR.setPower(driveMag * speeds[1]);
        bL.setPower(driveMag * speeds[2]);
        bR.setPower(driveMag * speeds[3]);

        telemetry.addData("frontLeft", fL.getPower());
        telemetry.addData("frontRight", fR.getPower());
        telemetry.addData("backLeft", bL.getPower());
        telemetry.addData("backRight", bR.getPower());

        lift(gamepad2.left_stick_y);

        telemetry.addData("Lift Speed", lift.getPower());

        if (gamepad1.left_bumper) intake(0.75);
        else if (gamepad1.right_bumper) intake(-0.75);
        else intake(0);

        double avgIntakePow = (intakeL.getPower() + intakeR.getPower()) / 2;
        telemetry.addData("Intake Speed", avgIntakePow);

        if (gamepad2.a) {
            foundationLever(0);
        }

        if (gamepad2.b) {
            foundationLever(1);
        }

        telemetry.addData("Foundation Lever Position", foundationL.getPosition());

        // moveRack(gamepad2.right_stick_y);
        /*
        double avgRackSpeed = (rackL.getPower() + rackR.getPower()) / 2;
        telemetry.addData("Rack Speed", avgRackSpeed);
         */

        if (gamepad2.x && !blockIsHeld) {
            setBlockHolder(0.5);
            blockIsHeld = true;
            try {
                Thread.sleep(1000);
            } catch (InterruptedException e) {
                telemetry.addData("Warning", "thread slept");
            }
        } else if (gamepad2.x) {
            setBlockHolder(1);
            blockIsHeld = false;
            try {
                Thread.sleep(1000);
            } catch (InterruptedException e) {
                telemetry.addData("Warning", "thread slept");
            }
        }

        telemetry.addData("Block Holder Position", blockHolder.getPosition());

        setGrabber(gamepad2.left_trigger);

        try {
            if (gamepad1.dpad_up && driveMag <= 0.9) {
                driveMag += 0.1;
                Thread.sleep(500);
            } else if (gamepad1.dpad_up) {
                driveMag = 0.1;
                Thread.sleep(500);
            } else if (gamepad1.dpad_down && driveMag >= 0.2) {
                driveMag -= 0.1;
                Thread.sleep(500);
            } else if (gamepad1.dpad_down) {
                driveMag = 1;
                Thread.sleep(500);
            } else Thread.sleep(0);
        } catch (InterruptedException e) {
            telemetry.addData("Warning","thread slept");
        }
    }

    public double[] driveMecanum(double x, double y, double turn) {
        double[] speeds = new double[4];

        double j = Math.hypot(x,y);
        double theta = Math.atan2(-y,x);

        speeds[0] = j * Math.sin(theta + Math.PI / 4) + turn;  // fL
        speeds[1] = j * Math.sin(theta - Math.PI / 4) - turn;  // fR
        speeds[2] = j * Math.sin(theta - Math.PI / 4) + turn;  // bL
        speeds[3] = j * Math.sin(theta + Math.PI / 4) - turn;  // bR

        return speeds;
    }

    public void lift(double pow) {
        lift.setPower(pow / 8);
    }

    public void intake(double pow) {
        intakeL.setPower(pow);
        intakeR.setPower(pow);
    }

    public void foundationLever(double pos) {
        foundationL.setPosition(pos);
        foundationR.setPosition(pos);
    }

    public void setBlockHolder(double pos) {
        blockHolder.setPosition(pos);
    }

    public void setGrabber(double pow) {
        grabber.setPower(pow);
    }

    /*
    public void moveRack(double pow) {
        rackL.setPower(pow / 2);
        rackR.setPower(pow / 2);
    }
     */
}