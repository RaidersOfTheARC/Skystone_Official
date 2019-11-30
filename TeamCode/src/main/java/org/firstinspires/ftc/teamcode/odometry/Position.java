package org.firstinspires.ftc.teamcode.Odometry;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.GyroSensor;

/**
 * This is the odometry positional coordinate that samples
 * the function of odometry via the return of an array that shall serve
 * as a vector containing the x, y, and heading of the robot.
 * Note that this is just as sample intended for practice for the
 * teams at ARC and is completely open source.
 * This code was made by Jackson Isenberg, ARC Programming Chair,
 * in 2019.
 */

public class Position {
    
  // let's assume we have two pods, one for vertical, and one for horizontal
  // therefore, this array is of length 2 ({horizontal, vertical})
  private DcMotor[] odoPods;
  private GyroSensor gyro;
  private double[] pos;
    
  public Position(DcMotor[] mots, GyroSensor gyr) {
    odoPods = mots;
    gyro = gyr;
    pos = new double[]{0,0,0};
    
    init();
  }
  
  public void init() {
    gyro.calibrate();
    
    for (DcMotor x : motors) {
        x.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        x.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
  }
  
  public double getX() {
    return pos[0];
  }
  
  public double getY() {
    return pos[1];
  }
  
  public double getHead() {
    return Math.toRadians(gyro.getHeading());
  }
    
  // assume prevEncoderVals is a two-element array {xi, yi}
  public void update(double[] prevEncoderVals) {
    double dx = odoPods[0].getCurrentPosition() - prevEncoderVals[0];
    double dy = odoPods[1].getCurrentPosition() - prevEncoderVals[1];
    pos[0] += dx*Math.cos(getHead()) - dy*Math.sin(getHead());
    pos[1] += dx*Math.sin(getHead()) + dy*Math.cos(getHead());
    pos[2] = getHead();
  }
  
  public double[] getPosition() {
    return pos;
  }

}
