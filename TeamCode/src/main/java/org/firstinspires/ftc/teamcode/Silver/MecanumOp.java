package org.firstinspires.ftc.teamcode.Silver;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * The practice mecanum teleop for Raiders of the ARC Silver,
 * team 10396. We are housed in Alpharetta High School, GA.
 * This code was made by Jackson Isenberg, ARC Programming Chair,
 * in 2019.
 */

@TeleOp(name="Drive Testing", group="Silver")
// @Disabled
public class MecanumOp extends OpMode {

    private CRServo rack;
    private StoneElevator lift;
    private Foundation foundationLever;
    private Intake intake;
    private Drivetrain drive;

    private double driveMag = 0.5;

    @Override
    public void init() {
        lift = new StoneElevator(hardwareMap.get(DcMotor.class, "lift"),
                                 hardwareMap.get(CRServo.class, "grabber"));

        intake = new Intake(hardwareMap.get(DcMotor.class, "intakeLeft"),
                            hardwareMap.get(DcMotor.class, "intakeRight"),
                            hardwareMap.get(Servo.class, "blockHolder"));

        drive = new Drivetrain(hardwareMap.get(DcMotor.class, "leftFront"),
                               hardwareMap.get(DcMotor.class, "rightFront"),
                               hardwareMap.get(DcMotor.class, "leftBack"),
                               hardwareMap.get(DcMotor.class, "rightBack"));

        foundationLever = new Foundation(hardwareMap.get(Servo.class, "foundationL"),
                                         hardwareMap.get(Servo.class, "foundationR"));

        rack = hardwareMap.get(CRServo.class, "rack");

        // rackL.setDirection(CRServo.Direction.FORWARD);
        // rackR.setDirection(CRServo.Direction.REVERSE);
    }

    @Override
    public void loop() {
        telemetry.addData("Drive Speed Multiplier", driveMag);

        double x = gamepad1.left_stick_x;
        double y = gamepad1.left_stick_y;
        double turn = gamepad1.right_stick_x;
        drive.drive(x, y, turn, driveMag);

        double[] speeds = drive.getSpeeds();
        telemetry.addData("frontLeft", speeds[0]);
        telemetry.addData("frontRight", speeds[1]);
        telemetry.addData("backLeft", speeds[2]);
        telemetry.addData("backRight", speeds[3]);

        lift.activate(gamepad2.left_stick_y * 0.1);
        if (gamepad2.left_bumper) lift.engage();
        else lift.disengage();

        telemetry.addData("Lift Speed", lift.getLiftSpeed());
        telemetry.addData("Grabber Active", lift.isEngaged());

        if (gamepad1.left_bumper) intake.activate(0.75);
        else if (gamepad1.right_bumper) intake.activate(-0.75);
        else intake.activate(0);

        telemetry.addData("Intake Speed", intake.getIntakeSpeed());

        try {
            if (gamepad2.a && !foundationLever.isActive()) {
                foundationLever.activate();
                Thread.sleep(500);
            }
            if (gamepad2.a && foundationLever.isActive()) {
                foundationLever.deactivate();
                Thread.sleep(500);
            }
        } catch (InterruptedException e) {
            telemetry.addData("You friggin moron", "you just got beaned");
        }

        telemetry.addData("Foundation Lever Active", foundationLever.isActive());

        // moveRack(gamepad2.right_stick_y);
        /*
        double avgRackSpeed = (rackL.getPower() + rackR.getPower()) / 2;
        telemetry.addData("Rack Speed", avgRackSpeed);
         */

        telemetry.addData("Block Holder Active", intake.isHeld());

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

    /*
    public void moveRack(double pow) {
        rackL.setPower(pow / 2);
        rackR.setPower(pow / 2);
    }
     */
}