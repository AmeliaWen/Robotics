package ca.mcgill.ecse211.project;


import static ca.mcgill.ecse211.project.Resources.*;
import static simlejos.ExecutionController.sleepFor;

import java.util.Arrays;

import static ca.mcgill.ecse211.project.LightLocalizer.*;
import static simlejos.ExecutionController.*;

import ca.mcgill.ecse211.playingfield.Point;

/**
 * The Navigation class is used to make the robot navigate around the playing field.
 */
public class Navigation {
 
  //The sample rate for average torque calculation
  private static final int TORQUE_SAMPLING_RATE = 500; 
  //Threshold for determining when the robot has left contact with
  // the block
  private static final double TORQUE_THRESHOLD = 0.15; 
  //An approximate 1% settling time, obtained from a research
  // paper on modelling the EV3 motor
  private static final double MOTOR_SETTLING_TIME = 3.0; 

  private static int numSamples = 0; // Counts the number of samples used in the torque average
  private static double torqueSum = 0; // Stores the sum of all torques to be averaged

  /** Do not instantiate this class. */
  private Navigation() {}

  private static final int SINTERVAL = 85;

  /** The number of samples in the moving median window. */
  private static final int MOVING_MEDIAN_SAMPLE_COUNT = 8;

  /** The distance at which the US readings are discarded because it is too far. */
  public static final int MIN_INVALID_DIST = 70;
  private static final int THRESHOLD = 255;
  private static final double OBSTACLE_DISTANCE = 2;
  //portion of the distance traversed before calculating torque
  public static final double TORQUE_PUSH_SPLIT = 1 / 4;
  public static final double[] LOCALIZE_MOVE = {-0.05, -0.025};
  private static int[] usMovingMedianData = new int[MOVING_MEDIAN_SAMPLE_COUNT];

  /** Current US Sensor value in cm. */
  private static int usSensorVal = 0;

  //Stores the 3 average torque values, one for each block
  public static double[] averageTorqueValues = new double[3]; 
  
  /**Pushes the blocks in proper order if they are in map 1 configuration.*/
  public static void map1() {
    simpleTravel2(new Point(2, 6));
    relocaliseAll();

    // move cube 1
    simplePushBlockTo(new Point(5.65, 6), 0);

    moveStraightFor(-0.65 * TILE_SIZE);
    relocaliseAll();

    // move cube 2
    simpleTravel2(new Point(5, 4));
    simplePushBlockTo(new Point(5, 1.35), 1);
    
    moveStraightFor(-0.65 * TILE_SIZE);
    simpleTravel2(new Point(5, 3));
    relocaliseAll();
    simpleTravel2(new Point(3, 3));
    simpleTravel2(new Point(3, 2));
    relocaliseAll();

    // move cube 3
    simplePushBlockTo(new Point(6.65, 2), 2);

    moveStraightFor(-0.65 * TILE_SIZE);
    relocaliseAll();

  }

  /**Pushes the blocks in proper order if they are in map 5 configuration.
   * 
   */
  public static void map5() {
    simpleTravel2(new Point(3, 1));
    relocaliseAll();
    
    simpleTravel2(new Point(6, 1));
    relocaliseAll();
    
    simpleTravel2(new Point(6, 4));
    relocaliseAll();
    
    simpleTravel2(new Point(6, 7));
    relocaliseAll();

    // move cube 1
    simpleTravel2(new Point(5, 6));
    simplePushBlockTo(new Point(2.55, 3.55), 1);
    moveStraightFor(-0.60 * TILE_SIZE);
    // simpleTravel2(new Point(3, 4));

    // moveStraightFor(-0.55 * TILE_SIZE);
    relocaliseAll();

    // move cube 2
    simpleTravel2(new Point(1, 4));
    relocaliseAll();
    simpleTravel2(new Point(1, 5.67));
    simpleTravel2(new Point(2, 5));
    simplePushBlockTo(new Point(4.67, 3.33), 0);
    moveStraightFor(-0.65 * TILE_SIZE);
    // simpleTravel2(new Point(4, 4));
    /*
     * moveStraightFor(-0.65 * TILE_SIZE); simpleTravel2(new Point(5, 3));
     *  relocaliseAll(); simpleTravel2(new Point(3,
     * 3)); simpleTravel2(new Point(3, 2)); relocaliseAll();
     * 
     * // move cube 3 simplePushBlockTo(new Point(6.65, 2), 2);
     * 
     * moveStraightFor(-0.65 * TILE_SIZE);
     */
    relocaliseAll();

  }
  
  /** Pushes the blocks in proper order if they are in map 6 configuration.
   * 
   */
  public static void map6() {
    simpleTravel2(new Point(1, 5));
    relocaliseAll();
    simpleTravel2(new Point(1.65, 5));
    simplePushBlockTo(new Point(4.65, 5), 0);
    moveStraightFor(-0.65 * TILE_SIZE);
    relocaliseAll();
    
    simpleTravel2(new Point(3, 6));
    relocaliseAll();
    simpleTravel2(new Point(3, 7));
    relocaliseAll();
    simpleTravel2(new Point(4, 7));
    relocaliseAll();
    simpleTravel2(new Point(4, 6.35));
    simplePushBlockTo(new Point(4, 2.35), 1);
    moveStraightFor(-0.65 * TILE_SIZE);
    relocaliseAll();
    
    
    simpleTravel2(new Point(4, 4));
    relocaliseAll();
    simpleTravel2(new Point(7, 4));
    relocaliseAll();
    simpleTravel2(new Point(7, 3));
    relocaliseAll();
    simpleTravel2(new Point(4.35, 3));
    simplePushBlockTo(new Point(3.35, 3), 2);
    moveStraightFor(-0.65 * TILE_SIZE);
    relocaliseAll();
  }

  /**Pushes the blocks in proper order if they are in map 2 configuration.*/
  public static void map2() {
    // Cube 2
    simpleTravel2(new Point(1, 3));
    relocaliseAll();
    simpleTravel2(new Point(5, 5));
    relocaliseAll();
    simpleTravel2(new Point(6, 5));
    simpleTravel2(new Point(6, 6));
    relocaliseAll();

    // push cube 2
    simpleTravel2(new Point(5, 6));
    simplePushBlockTo(new Point(2.35, 6), 1);

    moveStraightFor(-0.65 * TILE_SIZE);
    relocaliseAll();

    // Cube 3
    simpleTravel2(new Point(5, 2));
    relocaliseAll();

    // push cube 3
    simplePushBlockTo(new Point(1.35, 2), 2);

    moveStraightFor(-0.65 * TILE_SIZE);
    relocaliseAll();

    // Cube 1
    simpleTravel2(new Point(3, 2));
    relocaliseAll();

    // push cube 1
    simplePushBlockTo(new Point(3, 5.65), 0);

    moveStraightFor(-0.65 * TILE_SIZE);
    relocaliseAll();

  }
  
  /**Pushes the blocks in proper order if they are in map 3 configuration.
  * The waypoints should not be considered as "magic numbers"
  */
  public static void map3() {
    // Cube 1
    simpleTravel2(new Point(2, 2));
    relocaliseAll();

    // move cube 1
    //waypoints are not stored in a constant since they are determined from a map
    simplePushBlockTo(new Point(2, 5.65), 0); 

    moveStraightFor(-0.65 * TILE_SIZE);
    relocaliseAll();

    // Cube 2
    simpleTravel2(new Point(3, 3));
    relocaliseAll();
    simpleTravel2(new Point(4, 2));
    relocaliseAll();

    // move cube 2
    simplePushBlockTo(new Point(4, 5.65), 1);

    moveStraightFor(-0.65 * TILE_SIZE);
    relocaliseAll();

    // Cube 3
    simpleTravel2(new Point(6, 7));
    relocaliseAll();

    // move cube 3
    simplePushBlockTo(new Point(6, 3.35), 2);

    moveStraightFor(-0.65 * TILE_SIZE);
    relocaliseAll();
  }
  
  /**Pushes the blocks in proper order if they are in map 4 configuration.*/
  public static void map4() {
    // Cube 1
    simpleTravel2(new Point(0.6, 6));
    System.out.println("Moved to 0.6, 6");
    relocaliseY();
    simpleTravel2(new Point(0.6, 7));
    System.out.println("Moved to 0.6, 7");
    relocaliseY();

    // move cube 1
    simpleTravel2(new Point(2, 7));    
    moveStraightFor(-TILE_SIZE);
    relocaliseAll();

    // move cube 1
    simpleTravel2(new Point(2, 7));
    simplePushBlockTo(new Point(3.65, 7), 0);

    moveStraightFor(-0.65 * TILE_SIZE);
    relocaliseAll();

    // Cube 2
    simpleTravel2(new Point(4, 6));
    relocaliseAll();
    simpleTravel2(new Point(6, 7));
    relocaliseAll();
    simpleTravel2(new Point(6, 7.6));
    relocaliseX();
    simpleTravel2(new Point(7, 7.6));
    relocaliseX();

    // move cube 2
    simpleTravel2(new Point(7, 6.5));
    
    moveStraightFor(-0.5 * TILE_SIZE);
    relocaliseAll();

    // move cube 2
    simpleTravel2(new Point(7, 6));
    simplePushBlockTo(new Point(7, 4.35), 1);

    moveStraightFor(-0.65 * TILE_SIZE);
    relocaliseAll();

    // Cube 3
    simpleTravel2(new Point(6, 4));
    relocaliseAll();
    simpleTravel2(new Point(7, 2));
    relocaliseAll();
    simpleTravel2(new Point(7.6, 2));
    relocaliseY();
    simpleTravel2(new Point(7.6, 1));
    relocaliseY();

    // move cube 3
    simpleTravel2(new Point(6, 1));
    
    moveStraightFor(-TILE_SIZE);
    relocaliseAll();

    // move cube 3
    simpleTravel2(new Point(6, 1));
    simplePushBlockTo(new Point(4.35, 1), 2);

    moveStraightFor(-0.65 * TILE_SIZE);
    relocaliseAll();

  }
  
  /**Go to the closest grid intersection on the upper-right. The heading is restored to 
  /*the original one at the end of the process
  */
  public static void relocaliseAll() {
    Navigation.relocaliseX();
    Navigation.relocaliseY();
  }

  /** Travel through a straightline to a destination point.
   * @param destination point
   */
  public static void simpleTravel2(Point destination) {
    double x = destination.x;
    double y = destination.y;
    // difference in X coordinate in metres
    double deltaX = TILE_SIZE * x - Odometer.getOdometer().getXyt()[0];
    // difference in Y coordinate in metres
    double deltaY = TILE_SIZE * y - Odometer.getOdometer().getXyt()[1];
    // difference in X coordinate in tiles
    double deltaX1 = x - Math.round(Odometer.getOdometer().getXyt()[0] / TILE_SIZE);
    // difference in Y coordinate in tiles
    double deltaY1 = y - Math.round(Odometer.getOdometer().getXyt()[1] / TILE_SIZE);
    // heading needed to go to destination
    double finalTheta = Math.toDegrees(Math.atan2(deltaX, deltaY));
    turnTo(finalTheta);
    double distance = Math.sqrt(deltaX * deltaX + deltaY * deltaY);
    moveStraightFor(distance);
  }
  
  /**
   * Push the block from one grid intersection to another. Updates averageTorqueValues as it moves. 
   * @param destination destination point of the block
   * @Param index: the block number in the current map
   */
  public static void simplePushBlockTo(Point destination, int index) {
    double x = destination.x;
    double y = destination.y;
    double deltaX = TILE_SIZE * x - Odometer.getOdometer().getXyt()[0];
    double deltaY = TILE_SIZE * y - Odometer.getOdometer().getXyt()[1];
    double deltaX1 = x - Math.round(Odometer.getOdometer().getXyt()[0] / TILE_SIZE);
    double deltaY1 = y - Math.round(Odometer.getOdometer().getXyt()[1] / TILE_SIZE);
    double finalTheta = Math.toDegrees(Math.atan2(deltaX, deltaY));
    turnTo(finalTheta);
    double distance = Math.sqrt(deltaX * deltaX + deltaY * deltaY);
    //to calculate torque, the robot must already have a stable speed
    moveStraightFor(distance * TORQUE_PUSH_SPLIT); //let the robot go for a while
    moveStraightFor(distance * (1 - TORQUE_PUSH_SPLIT), true); //before starting to calculate torque
    calcAverageTorque(averageTorqueValues, index);
    
    // Correct for position errors due to torque calculation 
    deltaX = TILE_SIZE * x - Odometer.getOdometer().getXyt()[0];
    deltaY = TILE_SIZE * y - Odometer.getOdometer().getXyt()[1];
    distance = Math.sqrt(deltaX * deltaX + deltaY * deltaY);
    moveStraightFor(distance);
  }

  
  /**
   * Navigates to a destination while avoiding obstacles along the way. (not used, in development)
   * @param destination destination point
   */
  public static void simpleTravelObstacle(Point destination) {
    double x = destination.x;
    double y = destination.y;
    double initX = Odometer.getOdometer().getXyt()[0];
    double initY = Odometer.getOdometer().getXyt()[1];
    double deltaX = TILE_SIZE * x - initX;
    double deltaY = TILE_SIZE * y - initY;
    double deltaX1 = x - Math.round(Odometer.getOdometer().getXyt()[0] / TILE_SIZE);
    double deltaY1 = y - Math.round(Odometer.getOdometer().getXyt()[1] / TILE_SIZE);
    double finalTheta = Math.toDegrees(Math.atan2(deltaX1, deltaY1));
    turnTo(finalTheta);
    double distanceTraveled = 0;
    double distance = Math.sqrt(deltaX * deltaX + deltaY * deltaY);

    while (distanceTraveled < distance) {
      // System.out.println(readUsDistance());
      double deltaXf = Odometer.getOdometer().getXyt()[0] - initX;
      double deltaYf = Odometer.getOdometer().getXyt()[1] - initY;
      // if (Odometer.getOdometer().getXyt()[2] != finalTheta ) {
      // turnTo(finalTheta);
      // }
      distanceTraveled = Math.sqrt(deltaXf * deltaXf + deltaYf * deltaYf);
      int detect = readUsDistance();
      if (detect < 5 && detect > 0) {
        if (finalTheta <= 180) {
          turnTo(90);
          moveStraightFor(((Math.tan(Math.toRadians(finalTheta))) * 0.6 * TILE_SIZE) 
              + 0.5 * TILE_SIZE);
          turnBy(-90);
          moveStraightFor(0.6 * TILE_SIZE);
          turnBy(-90);
          moveStraightFor(0.5 * TILE_SIZE);
          turnBy(90 + finalTheta);
          clearUSSensor();
        } else {
          turnTo(-90);
          moveStraightFor(((Math.tan(Math.toRadians(finalTheta))) * 0.6 * TILE_SIZE) 
              + 0.5 * TILE_SIZE);
          turnBy(90);
          moveStraightFor(0.6 * TILE_SIZE);
          turnBy(90);
          moveStraightFor(0.5 * TILE_SIZE);
          turnBy(90 + finalTheta);
        }
        // System.out.println(detect);

      }
      leftMotor.setSpeed(FORWARD_SPEED);
      rightMotor.setSpeed(FORWARD_SPEED);
      leftMotor.forward();
      rightMotor.forward();
    }
    // double distance = Math.sqrt(deltaX * deltaX + deltaY * deltaY);
    // moveStraightFor(distance);



  }


  /**
   *  Relocalizes the bot to the next horizontal gridline. Adjust odometry as needed. 
   */
  public static void relocaliseY() {
    
    double curAngle = odometer.getXyt()[2];
    turnTo(0);
    moveStraightFor(LOCALIZE_MOVE[0]); //moves back a bit
    LightLocalizer.goToNextLine();
    odometer.setTheta(0);
    moveStraightFor(LOCALIZE_MOVE[1]);  //moves back more to prepare for rotation

    turnTo(curAngle);

    //odometry must show integer coordinate if robot is localized
    odometer.setY(TILE_SIZE * (Math.round(odometer.getXyt()[1] / TILE_SIZE))); 

    System.out.println("X : " + odometer.getXyt()[0] / TILE_SIZE + "   Y : " 
          + odometer.getXyt()[1] / TILE_SIZE);

  }

  /**
   * Relocalizes the bot to the next vertical gridline to the right. Adjust odometry as needed. 
   */
  public static void relocaliseX() {
    double curAngle = odometer.getXyt()[2];
    turnTo(90);
    moveStraightFor(LOCALIZE_MOVE[0]);
    LightLocalizer.goToNextLine();
    odometer.setTheta(90);
    moveStraightFor(LOCALIZE_MOVE[1]);
    turnTo(curAngle);
    odometer.setX(TILE_SIZE * (Math.round(odometer.getXyt()[0] / TILE_SIZE)));
    System.out.println("X : " + odometer.getXyt()[0] / TILE_SIZE + "   Y : " 
          + odometer.getXyt()[1] / TILE_SIZE);
  }

  /**
   * Turns the robot with a minimal angle towards the given input angle in degrees,
   *  no matter what its current orientation is. This method is different from {@code turnBy()}.
   *  @param angle in degrees. Where 0 is straight ahead. 90 degrees is to the right. 
   */
  public static void turnTo(double angle) {

    leftMotor.setSpeed(ROTATE_SPEED);
    rightMotor.setSpeed(ROTATE_SPEED);
    double turningAngle = minimalAngle(Odometer.getOdometer().getXyt()[2], angle);
    leftMotor.rotate(convertAngle(turningAngle), true);
    rightMotor.rotate(-convertAngle(turningAngle), false);

  }


  /** Returns the angle that the robot should point towards to face the destination in degrees. */
  public static double getDestinationAngle(Point current, Point destination) {
    double dx = destination.x - current.x;
    double dy = destination.y - current.y;
    double dt = Math.toDegrees(Math.atan2(dy, dx));
    if (dx < 0 && dy > 0) {
      return 360 - (dt - 90);
    }
    if (dx < 0 && dy < 0) {
      return -dt + 90;
    }
    if (dx > 0 && dy < 0) {
      return -dt + 90;
    }
    if (dx > 0 && dy > 0) {
      return 90 - dt;
    }
    if (dx == 0 && dy < 0) {
      return 180;
    }
    if (dx == 0 && dy > 0) {
      return 0;
    }
    if (dy == 0 && dx < 0) {
      return 270;
    }
    if (dy == 0 && dx > 0) {
      return 90;
    }
    return dt;
  }

  /** Returns the signed minimal angle in degrees from initial angle to destination angle (deg). */
  public static double minimalAngle(double initialAngle, double destAngle) {
    double turningAngle = destAngle - initialAngle;
    if (turningAngle > 180) {
      turningAngle -= 360;
    }
    // angle < 180, increase by 360
    if (turningAngle < -180) {
      turningAngle += 360;
    }
    return turningAngle;
  }

  /** Returns the distance between the two points in tile lengths (feet). */
  public static double distanceBetween(Point p1, Point p2) {
    return Math.sqrt((p2.x - p1.x) * (p2.x - p1.x) + (p2.y - p1.y) * (p2.y - p1.y));
  }

  
  // You can also add other helper methods here, but remember to document them with Javadoc (/**)!
  /**
   * Converts input distance to the total rotation of each wheel needed to cover that distance.
   * 
   * @param distance the input distance in meters
   * @return the wheel rotations necessary to cover the distance
   */
  public static int convertDistance(double distance) {
    return (int) ((distance * 180.0) / (Math.PI * WHEEL_RAD));
  }

  /**
   * Converts input angle to the total rotation of each wheel needed to 
   * rotate the robot by that angle.
   * 
   * @param angle the input angle in degrees
   * @return the wheel rotations necessary to rotate the robot by the angle
   */
  public static int convertAngle(double angle) {
    return convertDistance((angle * Math.PI * BASE_WIDTH) / 360.0);
  }

  /**
   * Turns the robot by a specified angle. Note that this method is different 
   * from {@code Navigation.turnTo()}. For example, if the robot is facing 90 degrees,
   *  calling {@code turnBy(90)} will make the robot turn to 180 degrees, but calling
   *  {@code Navigation.turnTo(90)} should do nothing (since the robot is already at 90 degrees).
   * 
   * @param angle the angle by which to turn, in degrees
   */

  public static void turnBy(double angle) {

    if (angle > 180) {
      angle -= 360;
    }
    // angle < 180, increase by 360
    if (angle < -180) {
      angle += 360;
    }
    int wheelAngle = convertAngle(angle);
    leftMotor.rotate(wheelAngle, true);
    rightMotor.rotate(-wheelAngle, false);
  }

  /**
   * Turns the robot to the desired angle.
   * 
   * @param angle The angle to which the robot should turn
   */
  public static void turnToOld(int angle) {
    angle = Math.abs(angle);
    angle = angle % 360;

    int currentAngle = (int) odometer.getXyt()[2];
    turnBy(angle - currentAngle);

  }

  /**
   * Moves the robot straight for the given distance.
   * 
   * @param distance in meters, may be negative
   */
  public static void moveStraightFor(double distance) {
    int angle = convertDistance(distance);
    leftMotor.rotate(angle, true);
    rightMotor.rotate(angle, false);
  }
  
  /**
   * Moves the robot straight for the given distance, returns immediately if true is passed.
   * 
   * @param distance in meters, may be negative
   * @param returnVal whether or not the method should return immediately
   */
  public static void moveStraightFor(double distance, boolean returnVal) {
    int angle = convertDistance(distance);
    leftMotor.rotate(angle, true);
    rightMotor.rotate(angle, returnVal);
  }

  /**
   * Stops both motors.
   */
  public static void stopMotors() {
    leftMotor.stop();
    rightMotor.stop();
  }

  /**
   * Starts both motors. (this method only calls forward() on them,
   *  it does not set speed or acceleration).
   */
  public static void startMotors() {
    leftMotor.forward();
    rightMotor.forward();
  }

  /**
   * Sets the speed of both motors.
   * 
   * @param speed The speed at which to set both motors
   */
  public static void setBothSpeed(int speed) {
    leftMotor.setSpeed(speed);
    rightMotor.setSpeed(speed);
  }

  /**
   * This method stops both motors and sets their speed to 0 degree/s.
   */
  public static void stopMotorsRemoveSpeed() {
    leftMotor.setSpeed(0);
    rightMotor.setSpeed(0);
    Navigation.stopMotors();
  }

  /**
   * This method will set the acceleration and speed of the motors to default values.
   */
  public static void prepareMotorsRotate() {
    leftMotor.setSpeed(ROTATE_SPEED);
    rightMotor.setSpeed(ROTATE_SPEED);
  }

  /**
  * Initializes a thread to calculate average torque, returns average torque to a given array index.
  * Assumes the method is only called when a change in torque is expected 
  */
  public static void calcAverageTorque(double[] arr, int index) {
    /* Reset variables */
    numSamples = 0;
    torqueSum = 0;
   
    double curTorque, prevTorque; // Stores current and previous torque, used in break case for while loop
    double timePassed = 0; // Used to ignore any torque values from within the MOTOR_SETTLING_TIME
     
    /* Initialize curTorque & prevTorque */
    curTorque = (rightMotor.getTorque() + leftMotor.getTorque())/2;
    prevTorque = curTorque;
    torqueSum += curTorque;
    numSamples++;

    /* Sums torque values while torque is within a threshold value */
    while (Math.abs(Math.abs(curTorque) - Math.abs(prevTorque)) < TORQUE_THRESHOLD
         || timePassed <= MOTOR_SETTLING_TIME) {
      sleepFor(TORQUE_SAMPLING_RATE); // Polls torque at 2 Hz
      timePassed += 0.5; // Increments the passed time

      /* Update torque values */
      prevTorque = curTorque;
      curTorque = (rightMotor.getTorque() + leftMotor.getTorque()) / 2;

      if (timePassed >= MOTOR_SETTLING_TIME) { // Ignore values if they are within the 
        //motor settling time
        torqueSum += curTorque;
        numSamples++;
      }

      System.out.println(leftMotor.getTorque() + " " + rightMotor.getTorque() + " " + curTorque);
    }
    
    System.out.println(torqueSum / numSamples);
    arr[index] = torqueSum / numSamples; // Print the average result to an array
  }

  private static int readUsDistance() {
    sleepFor(SINTERVAL);
    /* Shift array values over to make space for new sample */
    for (int i = MOVING_MEDIAN_SAMPLE_COUNT - 1; i > 0; i--) {
      usMovingMedianData[i] = usMovingMedianData[i - 1];
    }
    // Get the current sample, ensure it is within the range of accepted values
    usMovingMedianData[0] = Math.min(THRESHOLD, (int) fetchSample());
    usSensorVal = medianFilter(usMovingMedianData);
    return usSensorVal;
  }

  /**
   * Fetchs an unfiltered sample from the sensor.
   */
  private static float fetchSample() {
    float[] distValue = new float[usSensor.sampleSize()];
    usSensor.fetchSample(distValue, 0);
    return distValue[0] * 100;
  }

  /**
   * Performs a median filter on a data set. Uses bubble sort
   */
  public static int medianFilter(int[] data) {
    int median;
    int[] sortedData = new int[data.length];
    for (int i = 0; i < data.length; i++) {
      sortedData[i] = data[i]; // Copy values into new array
    }
    Arrays.sort(sortedData);
    if ((data.length) % 2 == 0) {
      median = (sortedData[data.length / 2] + sortedData[1 + data.length / 2]) / 2;
    } else {
      median = sortedData[data.length / 2];
    }
    return median;
  }
  
  /**
   * Performs a median filter on a data set. Uses bubble sort
   */
  public static double medianFilter(double[] data) {
    double median;
    double[] sortedData = new double[data.length];
    for (int i = 0; i < data.length; i++) {
      sortedData[i] = data[i]; // Copy values into new array
    }
    Arrays.sort(sortedData);
    if ((data.length) % 2 == 0) {
      median = (sortedData[data.length / 2] + sortedData[1 + data.length / 2]) / 2;
    } else {
      median = sortedData[data.length / 2];
    }
    return median;
  }
  
  /**
   * .
   * 
   */
  public static void clearUSSensor() {
    for (int i = 0; i < MOVING_MEDIAN_SAMPLE_COUNT; i++) {
      usMovingMedianData[i] = 255;
    }
  }
}
