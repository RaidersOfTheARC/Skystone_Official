package org.firstinspires.ftc.teamcode.odometry;

/**
 * For our odometry, we will not worry about explaining PID control for simplicity.
 * We want to control the robot by forcing it to drive to a certain position.
 * We can also control the heading and PIDF coefficients. These will be tuned with time,
 * but for now we will set them to be 1 for the sake of example (exempli gratia).
 * In order to do this, we will use the PID mode RUN_USING_ENCODER. Note that
 * the speeds rely on field-centric modeling.
 *
 * Made by you-know-who, so make sure to give me credit!
 */

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

public class RobotCommand {

  private DcMotorEx fL, fR, bL, bR;                   // drive motors
  private PIDFCoefficients pidf;                      // pidf coefficients
  private Position position;                          // current position on field

  // you need to cast DcMotor -> DcMotorEx
  public RobotCommand(DcMotor[] driveMotors, Position pos) {
    fL = (DcMotorEx)driveMotors[0];
    fR = (DcMotorEx)driveMotors[1];
    bL = (DcMotorEx)driveMotors[2];
    bR = (DcMotorEx)driveMotors[3];
    
    position = pos;
    
    init();
  }
  
  /**
   * Here, we are initiating our PIDF coefficients to the initial constructor
   */
  public void init() {
    pidf = new PIDFCoefficients();           // change this and tune values
    
    fL.setVelocityPIDFCoefficients(pidf.p, pidf.i, pidf.d, pidf.f);
    fR.setVelocityPIDFCoefficients(pidf.p, pidf.i, pidf.d, pidf.f);
    bL.setVelocityPIDFCoefficients(pidf.p, pidf.i, pidf.d, pidf.f);
    bR.setVelocityPIDFCoefficients(pidf.p, pidf.i, pidf.d, pidf.f);
  }
  
  // sets the target position (in ticks) and returns the current horizontal and vertical distance
  public double[] setTarget(double[] tgt) {    
    double xDist = tgt[0] - getCurrentX();
    double yDist = tgt[1] - getCurrentY();
    
    return new double[]{xDist, yDist};
  }
  
  public double getCurrentX() {
    return position.getX();
  }
  
  public double getCurrentY() {
    return position.getY();
  }
  
  public double[] driveToTarget(double[] tgt) {
    double[] displacement = setTarget(tgt);
    double tolerance = 0;     // tune this as you please

    /*
      As you can see, what this does is cause the speed to slow down
      as the robot nears its target tolerance. Make sure to tune the values
      for the PIDF coefficients so that the movement is more accurate.
    */
    while (displacement[0] > tolerance && displacement[1] > tolerance) {
      double[] speeds = driveMecanum(displacement[0], displacement[1], 0);
      double[] prevPosits = new double[]{getCurrentX(), getCurrentY()};
      
      fL.setVelocity(speeds[0]);
      fR.setVelocity(speeds[1]);
      bL.setVelocity(speeds[2]);
      bR.setVelocity(speeds[3]);
            
      position.update(prevPosits);
      displacement = setTarget(tgt);
    }
    
    return position.getPosition();  // use this for telemetry to see if the target position has been reached
  }

  public double[] turnByAngle(double rad) {
    double angleTgt = rad + position.getHead();
    double tolerance = 0;    // tune this value as you please

    /*
      As you can see, this turns the robot to a calculated turn target.
      We use an additional P control to decrease the speed as it nears the angle given
      a certain tolerance, if needed. As the heading nears the target, the speed will
      decrease on top of the internal PIDF control in the DcMotorEx;
    */
    while (position.getHead() < angleTgt + tolerance) {
      double[] prevPosits = new double[]{getCurrentX(), getCurrentY()};
      double[] speeds = driveMecanum(0, 0, angleTgt - tolerance - position.getHead());

      fL.setVelocity(speeds[0]);
      fR.setVelocity(speeds[1]);
      bL.setVelocity(speeds[2]);
      bR.setVelocity(speeds[3]);

      position.update(prevPosits);
    }

    return position.getPosition();  // use this for telemetry to ensure heading is within tolerance
  }

  public double[] driveWhileTurning(double[] tgt, double rad) {
    double angleTgt = rad + position.getHead();
    double[] displacement = setTarget(tgt);
    double tolerance = 0;    // tune as you please
    double angleTol = 0;     // tune as you please

    boolean hasTurned = false;

    while (displacement[0] > tolerance && displacement[1] > tolerance) {
      double[] prevPosits = new double[]{getCurrentX(), getCurrentY()};
      double turnVal = angleTgt - angleTol - position.getHead();
      if (Math.abs(turnVal) <= angleTol && !hasTurned) hasTurned = true;
      if (hasTurned) turnVal = 0;  // this is really just to ensure that turning stops
      double[] speeds = driveMecanum(displacement[0], displacement[1], turnVal);

      fL.setVelocity(speeds[0]);
      fR.setVelocity(speeds[1]);
      bL.setVelocity(speeds[2]);
      bR.setVelocity(speeds[3]);

      position.update(prevPosits);
    }

    return position.getPosition();
  }

  private double[] driveMecanum(double x, double y, double turn) {
    double[] speeds = new double[4];

    double j = Math.hypot(x,y);
    double theta = Math.atan2(-y,x) - position.getHead();

    speeds[0] = j * Math.sin(theta + Math.PI / 4) + turn;  // fL
    speeds[1] = j * Math.sin(theta - Math.PI / 4) - turn;  // fR
    speeds[2] = j * Math.sin(theta - Math.PI / 4) + turn;  // bL
    speeds[3] = j * Math.sin(theta + Math.PI / 4) - turn;  // bR

    return speeds;
  }

}
