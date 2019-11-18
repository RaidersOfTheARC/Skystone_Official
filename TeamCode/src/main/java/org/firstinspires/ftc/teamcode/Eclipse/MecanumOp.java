package org.firstinspires.ftc.teamcode.Eclipse;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Silver.Drivetrain;

/**
 * The practice mecanum teleop for Team Eclipse of ARC,
 * team 12864. We are housed in Alpharetta High School, GA.
 * This code was made by Jackson Isenberg, ARC Programming Chair,
 * in 2019.
 */

@TeleOp(name="Drive Practice", group="Eclipse")
// @Disabled
public class MecanumOp extends OpMode {

    private Drivetrain robotDrive;

    private double driveMag = 0.5;

    @Override
    public void init() {
        robotDrive = new Drivetrain(hardwareMap.get(DcMotor.class, "fl"),
                                    hardwareMap.get(DcMotor.class, "fr"),
                                    hardwareMap.get(DcMotor.class, "bl"),
                                    hardwareMap.get(DcMotor.class, "br"));
    }

    @Override
    public void loop() {
        telemetry.addData("Drive Speed Multiplier", driveMag);

        double x = gamepad1.left_stick_x;
        double y = gamepad1.left_stick_y;
        double turn = gamepad1.right_stick_x;
        robotDrive.drive(x, y, turn, driveMag);

        double[] speeds = robotDrive.getSpeeds();
        telemetry.addData("frontLeft", speeds[0]);
        telemetry.addData("frontRight", speeds[1]);
        telemetry.addData("backLeft", speeds[2]);
        telemetry.addData("backRight", speeds[3]);

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
}
