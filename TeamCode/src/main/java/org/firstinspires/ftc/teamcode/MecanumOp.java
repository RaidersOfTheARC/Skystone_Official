package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="Drive Testing")
// @Disabled
public class MecanumOp extends OpMode {

    private DcMotor fL, fR, bL, bR;
    private Servo foundationL, foundationR;
    private CRServo rackL, rackR;
    private StoneElevator lift;
    private Intake intake;

    private double driveMag = 0.5;
    private boolean blockIsHeld = false;

    @Override
    public void init() {
        fL = hardwareMap.get(DcMotor.class, "leftFront");
        fR = hardwareMap.get(DcMotor.class, "rightFront");
        bL = hardwareMap.get(DcMotor.class, "leftBack");
        bR = hardwareMap.get(DcMotor.class, "rightBack");

        lift = new StoneElevator(hardwareMap.get(DcMotor.class, "lift"),
                                 hardwareMap.get(CRServo.class, "grabber"));

        intake = new Intake(hardwareMap.get(DcMotor.class, "intakeLeft"),
                            hardwareMap.get(DcMotor.class, "intakeRight"),
                            hardwareMap.get(Servo.class, "blockHolder"));

        foundationL = hardwareMap.get(Servo.class, "foundationL");
        foundationR = hardwareMap.get(Servo.class, "foundationR");
        rackL = hardwareMap.get(CRServo.class, "forward1");
        rackR = hardwareMap.get(CRServo.class, "forward2");

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

        foundationL.setDirection(Servo.Direction.FORWARD);
        foundationR.setDirection(Servo.Direction.REVERSE);
        // rackL.setDirection(CRServo.Direction.FORWARD);
        // rackR.setDirection(CRServo.Direction.REVERSE);

        foundationL.scaleRange(0, 0.8);
        foundationR.scaleRange(0, 0.8);
        foundationL.setPosition(0);
        foundationR.setPosition(0);
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

        lift.activate(gamepad2.left_stick_y * 0.1);
        if (gamepad2.left_bumper) lift.engage();
        else lift.disengage();

        telemetry.addData("Lift Speed", lift.getLiftSpeed());
        telemetry.addData("Grabber Active", lift.isEngaged());

        if (gamepad1.left_bumper) intake.activate(0.75);
        else if (gamepad1.right_bumper) intake.activate(-0.75);
        else intake.activate(0);

        telemetry.addData("Intake Speed", intake.getIntakeSpeed());

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

        telemetry.addData("Block Holder Position", intake.isHeld());

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

    public void foundationLever(double pos) {
        foundationL.setPosition(pos);
        foundationR.setPosition(pos);
    }

    /*
    public void moveRack(double pow) {
        rackL.setPower(pow / 2);
        rackR.setPower(pow / 2);
    }
     */
}