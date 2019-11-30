package org.firstinspires.ftc.teamcode.Silver;

/**
 * A sample auto template that uses odometry. Using this,
 * we can create a simple yet powerful autonomous program.
 * You already know who made this, so give me credit please.
 * I wrote this at 4 AM because I was bored.
 */

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.GyroSensor;

import org.firstinspires.ftc.teamcode.odometry.Position;
import org.firstinspires.ftc.teamcode.odometry.RobotCommand;

@Autonomous(name="Template Auto", group="Silver")
// @Disabled
public class AutoTemplate extends LinearOpMode {

    private static final double COUNTS_PER_REV = 1000;   // ticks per revolution for odometry pods
    private static final double ODO_WHEEL_DIAM = 3;      // for this example, this is in inches
    private RobotCommand commander;
    private Position robotPosition;
    private DcMotor[] deadWheels, driveMotors;
    private GyroSensor gyro;

    @Override
    public void runOpMode() throws InterruptedException {
        robotPosition = new Position(deadWheels, gyro);
        commander = new RobotCommand(driveMotors, robotPosition);

        waitForStart();

        double[] target_1 = convertTargetToTicks(new double[]{2,3});
        commander.driveToTarget(target_1);
        telemetry.addData("Robot Position", robotPosition);
        /**
         * Alternatively, you can use:
         * @code    telemetry.update("Robot Position", commander.driveToTarget(target_1));
         */
        telemetry.update();

        commander.turnByAngle(Math.toRadians(30));
        telemetry.addData("Robot Position", robotPosition);
        telemetry.update();

        double[] target_2 = convertTargetToTicks(new double[]{-3,6});
        commander.driveToTarget(target_2);
        telemetry.addData("Robot Position", robotPosition);
        telemetry.update();

        double[] target_3 = convertTargetToTicks(new double[]{0,0});
        commander.driveWhileTurning(target_3, Math.toRadians(-30));
    }

    public double[] convertTargetToTicks(double[] tgt) {
        tgt[0] *= COUNTS_PER_REV / (Math.PI * ODO_WHEEL_DIAM);
        tgt[1] *= COUNTS_PER_REV / (Math.PI * ODO_WHEEL_DIAM);

        return tgt;
    }

}
