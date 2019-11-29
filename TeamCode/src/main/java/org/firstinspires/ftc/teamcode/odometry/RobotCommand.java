package org.firstinspires.ftc.teamcode.Odometry;

/**
 * For our odometry, we will not worry about explaining PID control for simplicity.
 * We want to control the robot by forcing it to drive to a certain position.
 * We can also control the heading and PIDF coefficients. These will be tuned with time,
 * but for now we will set them to be 1 for the sake of example (exempli gratia).
 * In order to do this, we will use the PID mode RUN_USING_ENCODER with an additional
 * P control through a method known as gradient speed adjustment.
 * In order to create this P control, we just need to account for the proportional error.
 * This is done by doing error * kP. Our error will be nothing but our proportional distance from the
 * target and the kP value will be such that is fine-tuned to produce more accurate results.
 * The only change we will make is that the additional P loop will use a kP value of
 * 1/targetTicks.
 */

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients

public class RobotCommand {

  DcMotorEx fL, fR, bL, bR;
  DcMotor oHoriz, oVertic;
  PIDFCoefficients pidf;
  double kp;

  public RobotCommand(DcMotor[] driveMotors, DcMotor[] deadWheels) {
    fL = driveMotors[0];
    fR = driveMotors[1];
    bL = driveMotors[2];
    bR = driveMotors[3];
    
    oHoriz = deadWheels[0];
    oVertic = deadWheels[1];
    
    init();
  }
  
  /**
   * Here, we are initiating our PIDF coefficients to be 1, 1, 1, 1.
   * Our initial kp will be 0, but that will change when we are given a target position.
   */
  public void init() {
    pidf = new PIDCoefficients(1,1,1,1);
    kp = 0;
    fL.setVelocityPIDFCoefficients(pidf);
    fR.setVelocityPIDFCoefficients(pidf);
    bL.setVelocityPIDFCoefficients(pidf);
    bR.setVelocityPIDFCoefficients(pidf);
  }

}
