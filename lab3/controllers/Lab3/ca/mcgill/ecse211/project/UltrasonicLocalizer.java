package ca.mcgill.ecse211.project;
import static ca.mcgill.ecse211.project.Resources.*;
import static simlejos.ExecutionController.sleepFor;

import java.util.Arrays;

public class UltrasonicLocalizer {
  private static float[] usData = new float[usSensor.sampleSize()];
  /**
   * The number of invalid samples seen by {@code filter()} so far.
   */
  private static final int SINTERVAL = 85;
  private static final int MOVING_MEDIAN_SAMPLE_COUNT = 8;
  private static final int THRESHOLD = 255;
  private static final int LIGHT_SENSOR_THRESHOLD = 12;
  private static int [] usMovingMedianData = new int[MOVING_MEDIAN_SAMPLE_COUNT];
  private static int usSensorVal = 0; // Current sensor value in cm
  private static int d = 30; // Wall distance 
  private static int k = 1;  //Gap distance 
      
  private static UltrasonicLocalizer ultraLoco;
      
  //private static double theta;
  private static Odometer odometer;
  
  /* A private constructor */
  private UltrasonicLocalizer() {
  }
  
  /**
   * getUltrasonicLocalizer method.    
   * @return
   */
  public static UltrasonicLocalizer getUltrasonicLocalizer() {
    if (ultraLoco == null) {
      ultraLoco = new UltrasonicLocalizer();
      odometer = Odometer.getOdometer();
    }
    return ultraLoco;
  }
  
  /**
   * localize: main method to run us sensor.
   */
  public static void localize() {
    odometer = Odometer.getOdometer();
    initSensor();
    
    System.out.println(updateUsSensor());
    if (updateUsSensor() < 2 * d) {
      Driver.setSpeed(ROTATE_SPEED);
      while (updateUsSensor() < 2 * d) {
        leftMotor.forward();
        rightMotor.backward();
      }
      Driver.stopMotors();          
    }
    System.out.println("Beginning Falling Edge localization");
    fallingEdge();
  }
  
  /**
   * Move the robot inside the square for Light Localization.
   */
  public static void orientForLsLocalization() {
    initSensor();
    if (updateUsSensor() > LIGHT_SENSOR_THRESHOLD) {
      System.out.println(usSensorVal);
      LightLocalizer.reverseUntilLine();
    }
    
    Driver.setSpeed(ROTATE_SPEED);
    Driver.turnBy(90);
    
    initSensor();
    if (updateUsSensor() > LIGHT_SENSOR_THRESHOLD) {
      System.out.println(usSensorVal);
      LightLocalizer.reverseUntilLine();
    }
    
    Driver.stopMotors();
  }
  
  /**
   * Implements a falling edge detection algorithm to calculate the appropriate heading.
   */
  private static void fallingEdge() {   
    double a1, a2, alpha;  //Angle to the back
    double b1, b2, beta;  //Angle to the left 
    double deltaTheta;  //Angle to add to orient the robot 
    leftMotor.setSpeed(ROTATE_SPEED);
    rightMotor.setSpeed(ROTATE_SPEED);

    // Rotate until we see a wall 
    while (updateUsSensor() > d + k) {
      leftMotor.forward();
      rightMotor.backward();
    }
    a1 = odometer.getXyt()[2];

    // Keep Rotating until we see a wall
    while (updateUsSensor() > d - k) {
      rightMotor.backward();
      leftMotor.forward();
    }
    a2 = odometer.getXyt()[2];
    
    // Record angle of left wall
    alpha = (a1 + a2) / 2; 
    leftMotor.setSpeed(ROTATE_SPEED);
    rightMotor.setSpeed(ROTATE_SPEED);
    
    //switch direction and wait until we don't see a wall
    while (updateUsSensor() < 2 * d) {
      leftMotor.backward();
      rightMotor.forward();
    }    
    // Keep Rotating until we see a wall
    while (updateUsSensor() > d + k) {
      leftMotor.backward();
      rightMotor.forward();
    }
    b1 = odometer.getXyt()[2];

    // Keep Rotating until we see a wall
    while (updateUsSensor() > d - k) {
      leftMotor.backward();
      rightMotor.forward();
    }
    b2 = odometer.getXyt()[2];

    //Record angle alpha of back wall
    beta = (b1 + b2) / 2;
    Driver.stopMotors();
       
    // Calculate angle we need to rotate
    deltaTheta = getTheta(alpha, beta);
    
    // Rotate and correct for light sensor position
    Driver.setSpeed(ROTATE_SPEED);
    Driver.turnBy(deltaTheta - 90);
    Driver.stopMotors();
    
    odometer.setXyt(0, 0, 0);
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
  private static int updateUsSensor() {
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
      
  private static double getTheta(double alpha, double beta) {
    double theta = 0;
    if (alpha < beta) {
      theta = 45 - (alpha - beta) / 2;
    } else if (alpha > beta) {
      theta = 225 - (alpha - beta) / 2; 
    }
    return theta;
  }
}