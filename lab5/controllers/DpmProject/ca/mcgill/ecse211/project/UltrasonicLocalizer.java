package ca.mcgill.ecse211.project;

import static ca.mcgill.ecse211.project.Resources.MAX_SENSOR_DIST;
import static ca.mcgill.ecse211.project.Resources.leftMotor;
import static ca.mcgill.ecse211.project.Resources.odometer;
import static ca.mcgill.ecse211.project.Resources.rightMotor;
import static ca.mcgill.ecse211.project.Resources.usSensor;
import static simlejos.ExecutionController.sleepFor;

import java.util.Arrays;

public class UltrasonicLocalizer {

  /** The interval between US Sensor samples. */
  private static final int SINTERVAL = 85;
  
  /** The number of samples in the moving median window. */
  private static final int MOVING_MEDIAN_SAMPLE_COUNT = 8;
  
  /** The distance at which the US readings are discarded because it is too far. */
  public static final int MIN_INVALID_DIST = 70;
  
  /** The threshold distance for which US Sensor are considerd invalid. */
  private static final int THRESHOLD = 255;
  
  /** Minimum angle between 2 walls. */
  private static final double MIN_ANGLE_WALL = 60;
  
  /** The angle by which the robot was last turn before stopping. */
  private static int lastAngle = 0;
  
  /** Stores the data for the moving median filter. */
  private static int [] usMovingMedianData = new int[MOVING_MEDIAN_SAMPLE_COUNT];
  
  /** Current US Sensor value in cm. */
  private static int usSensorVal = 0;
  
  /**
   * This method localizes the robot's orientation.
   */
  public static void localize() {

    initSensor();
    Navigation.prepareMotorsRotate();
    Navigation.startMotors();
    prepareRisingEdge();
    odometer.setTheta(0);
    turnClockwise();
    untilSeeNoWall(false);
    int angleA = 0;
    angleA = (int) odometer.getXyt()[2];
    lastAngle = angleA;
    turnCounterClockwise();
    untilSeeWall(true);
    turnCounterClockwise();
    untilSeeNoWall(true);
    int angleB = 0;
    angleB = (int) odometer.getXyt()[2];
    int angle = calibrateAngle(angleA, angleB);
    Navigation.turnToOld(angle - 45);
  }
  
  /**
   * This method will place the robot in an adequate position to execute rising edge instructions.
   */
  private static void prepareRisingEdge() {
    //fill the sample array (because if unlucky, could return a noise value)
    for (int i = 0; i < MOVING_MEDIAN_SAMPLE_COUNT; i++) {
      readUsDistance(); 
    }
    int dist = readUsDistance();
    //turn until the robot sees a wall
    while (dist > MAX_SENSOR_DIST) {
      //the difference in degree between 2 samples
      Navigation.turnBy(MIN_ANGLE_WALL);
      //refill the array (only 1 sample per min angle rotation, so refill is necessary)
      for (int i = 0; i < MOVING_MEDIAN_SAMPLE_COUNT; i++) {
        readUsDistance(); 
      }
      dist = readUsDistance();
    }
    Navigation.stopMotorsRemoveSpeed();
  }
  
  /** 
   * This method will run until a wall can be seen.
   * @param minAngleWall true if the method should watch out for a minimum angle between walls.
   */
  private static void untilSeeWall(boolean minAngleWall) {
    if (minAngleWall) {
      while (readUsDistance() > MAX_SENSOR_DIST 
          || Math.abs(odometer.getXyt()[2] - lastAngle) < MIN_ANGLE_WALL) {
      }
      return;
      
    } else {
      while (readUsDistance() > MAX_SENSOR_DIST) {
      }
      return;
    }
  }
  
  /** 
   * This method will run until a wall it not seen.
   * @param minAngleWall true if the method should watch out for a minimum angle between walls.
   */
  private static void untilSeeNoWall(boolean minAngleWall) {
    if (minAngleWall) {
      while (readUsDistance() < MAX_SENSOR_DIST
           || Math.abs(odometer.getXyt()[2] - lastAngle) < MIN_ANGLE_WALL) {
      }
      return;
    
    } else {
      while (readUsDistance() < MAX_SENSOR_DIST) {
      }
      return;
    }
  }
  
  /**
   * This robot will turn the robot in a clockwise manner.
   */
  private static void turnClockwise() {
    Navigation.prepareMotorsRotate();
    leftMotor.forward();
    rightMotor.backward();
  }
  
  /**
   * This method will turn the robot in an anti-clockwise manner.
   */
  private static void turnCounterClockwise() {
    Navigation.prepareMotorsRotate();
    leftMotor.backward();
    rightMotor.forward();
  }
  
  /**
   * Takes 2 angles and gives the average in the interval [0,360].
   * @param angleA The first angle.
   * @param angleB The second angle.
   */
  private static int calibrateAngle(int angleA, int angleB) {
    int angleC = (angleA + angleB) / 2;
    return angleC % 360;
  }

  /**
   * Initializes the sensor array values.
   */
  private static void initSensor() {
    for (int i = 0; i < MOVING_MEDIAN_SAMPLE_COUNT; i++) {
      usMovingMedianData[i] = (int) fetchSample();
      sleepFor(SINTERVAL);
    }
    usSensorVal = medianFilter(usMovingMedianData);
  }
      
  /**
   * Returns the filtered distance between the US sensor and an obstacle in cm.
   * 
   * @return the filtered distance between the US sensor and an obstacle in cm
   */
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
     * Performs a median filter on a data set.
     * Uses bubble sort
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
}
