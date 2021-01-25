package ca.mcgill.ecse211.project;
import static ca.mcgill.ecse211.project.Resources.*;
import static simlejos.ExecutionController.sleepFor;

public class LightLocalizer {
  // Sampling interval in mS
  public static final int SINTERVAL = 30; 
  //The number of samples considered in the moving average
  public static final int SAMPLE_AVERAGE_NUM = 5; 
  //The distance the robot reverses after contacting the first line
  public static final double REVERSE_DISTANCE = 0.1; 
  //The threshold over which we consider a value read to be considered as a line
  public static final double THRESHOLD = 0.55; 
  //How far the robot must overshoot a line to ensure the sensor detect the line when localizing
  public static final double LINE_OVERSHOOT = 0.2; 
  //Moving average of light sensor inputs
  public static double lightSensorVal = 0; 
  //The previous value of the moving average
  public static double lightSensorValPrev = 0; 
  //Array containing the values over which the moving average is being calculated
  private static double[] lightSensorAvgVals = new double[SAMPLE_AVERAGE_NUM]; 
  //Stores the derivative of the light sensor signal at the current point
  private static double lightSensorDerivative; 
  //Flags when our robot has detected a line
  private static boolean lineDetected = false; 
  //Flags when our robot is localized
  public static boolean isLocalized = false; 
  private static LightLocalizer lightLoco;
  //Allocate buffer for colour sensor
  static float[] sampleColor = new float[colorSensor.sampleSize()]; 
  
  /* Base Constructor */
  private LightLocalizer() {
  }
  
  /**
   * Returns an instance of this object.
   * @return
   */
  public static LightLocalizer getLightLocalizer() {
    if (lightLoco == null) {
      lightLoco = new LightLocalizer();
    }
    return lightLoco;
  }
  
  /**
   * Runs the localization algorithm.
   */
  public static void localize() {
    initLightSensor();
    UltrasonicLocalizer.orientForLsLocalization(); // Orient the robot
    System.out.println("Starting light localization algorithm");
    moveToIntersection(); // Move to near the (1,1) intersection
    // Determine the amount we need to correct our position by
    double [] positionDeltas = determineHeadingAndPosition(); 
    /* Correct position */
    Driver.stopMotors();
    Driver.setSpeed(FORWARD_SPEED);
    Driver.moveStraightFor(-(1 - positionDeltas[1] + LIGHT_SENSOR_DISTANCE));
    Driver.setSpeed(ROTATE_SPEED);
    Driver.turnBy((-90 - positionDeltas[2] * 180 / Math.PI));
    Driver.setSpeed(FORWARD_SPEED);
    Driver.moveStraightFor(-(1 - positionDeltas[0]));
    isLocalized = true;
    rightMotor.stop(true);
    rightMotor.stop();
  }
  
 
  /**
  * Moves the robot until it crosses a line, use in ultrasonic localization.
  */
  public static void reverseUntilLine() {
    /* Moves forward until a line is detected */
    Driver.setSpeed(FORWARD_SPEED);
    leftMotor.forward();
    rightMotor.forward();
    findLineStart();
   
    /* Move over the line to allow the robot to intersect each of the lines when localizing */
    Driver.stopMotors();
    Driver.setSpeed(FORWARD_SPEED);
    Driver.moveStraightFor(LINE_OVERSHOOT);
  }
 
  /**
    * Moves the robot to a point close to the intersection of two black lines to 
    * allow the localization algorithm to run.
  */
  private static void moveToIntersection() {
    /* Moves the robot forward until a line is detected */
    Driver.setSpeed(FORWARD_SPEED);
    Driver.moveForward();
    findLineStart();
    /* Move over the line to allow the robot to intersect each of the lines when localizing */
    Driver.stopMotors();
    Driver.setSpeed(FORWARD_SPEED);
    Driver.moveStraightFor(-LINE_OVERSHOOT);
    /* Turn 90 degrees CW */
    Driver.setSpeed(ROTATE_SPEED);
    Driver.turnBy(-90);
    /* Moves the robot forward until a line is detected */
    Driver.setSpeed(FORWARD_SPEED);
    Driver.moveForward();
    findLineStart();
    /* Move over the line to allow the robot to intersect each of the lines when localizing */
    Driver.stopMotors();
    Driver.setSpeed(FORWARD_SPEED);
    Driver.moveStraightFor(-LINE_OVERSHOOT);
  }
   
  /**
    * Rotates the robot 360 degrees 
    * and uses the obtained data to calculate its position and heading.
    * Array contents: {deltax, deltay, deltatheta}
    * @return an array containing the delta values for x, y, and theta. 
    */
  private static double[] determineHeadingAndPosition() {
    double[] thetaVals = new double[4];
    /* Start a thread that will track the theta values of the lines when they are crossed */
    new Thread(() -> {
      for (int i = 0; i < 4; i++) {
        findLineStart();
        thetaVals[i] = ((Odometer.getOdometer()).getXyt())[2];
        detectLineCrossing();
      }
    }).start();
    /* Rotate the robot 360 degrees */
    Driver.setSpeed(ROTATE_SPEED);
    rightMotor.rotate(Driver.convertAngle(450), true);
    leftMotor.rotate(-Driver.convertAngle(450), false);
    /* These calculations are specific to how our robot aligns itself */
    double thetax = (Math.abs(thetaVals[2] - thetaVals[0])) * Math.PI / 180; // In radians
    double thetay = (Math.abs(thetaVals[3] - thetaVals[1])) * Math.PI / 180; // In radians
    /* Adjust thetax, thetay if they are too large */
    if (thetax > Math.PI) {
      thetax = 2 * Math.PI - thetax;
    }
    if (thetay > Math.PI) {
      thetay = 2 * Math.PI - thetay;
    }
    /* Calculate x and y position */
    double x = 1 - LIGHT_SENSOR_DISTANCE * Math.cos(thetay / 2); // With respect to (1,1)
    double y = 1 - LIGHT_SENSOR_DISTANCE * Math.cos(thetax / 2); // With respect to (1,1)    
    /* Calculate error in heading */
    double deltathetay = Math.abs(thetay / 2 - (thetaVals[3]) * Math.PI / 180);
    double deltatheta = deltathetay; // With respect to 0 degrees
    /* Return results */
    double [] returnArr = {x, y, deltatheta};
    return returnArr;
  }
   
  /**
    * Detects when our robot has crossed a line.
   */
  private static void detectLineCrossing() {
    boolean lineCrossed = false;
    while (!lineCrossed) {
      updateLightSensor();
      if (lightSensorDerivative > THRESHOLD) {
        sleepFor(100);
        lineCrossed = true;
      }
      sleepFor(SINTERVAL);
    }
  }
  
  /**
   * findLineStart.
   * Drives the robot forward until a line is detected, then returns from the method
   */
  private static void findLineStart() {
    lineDetected = false; // Assume method is only called when a line is not detected
    while (!lineDetected) {
      updateLightSensor();
      lineDetected = isLine();
      sleepFor(SINTERVAL); // Take a sample only once every 15 mS
    }
  }
  
  /**
   * isLine.
   * Checks light sensor values to determine if a line has been detected
   * @return True if a line has been detected, false otherwise
   */
  private static boolean isLine() {
    // Compares derivative to determined threshold value
    if (lightSensorDerivative < -THRESHOLD) {
      return true;
    }
    return false;
  }
  
  /**
   * updateLightSensor.
   * Updates the Light Sensor moving average based on the most recent sample
   */
  private static void updateLightSensor() {
    // Shifts the array values by 1 to make space for the new sample
    for (int i = SAMPLE_AVERAGE_NUM - 1; i > 0; i--) { 
      lightSensorAvgVals[i] = lightSensorAvgVals[i - 1];
    }
    lightSensorAvgVals[0] = fetchSample(); // Get a new sample
    lightSensorValPrev = lightSensorVal;
    // Calculate the mean of the new sample set
    lightSensorVal = lightSensorVal + (lightSensorAvgVals[0] - lightSensorAvgVals[SAMPLE_AVERAGE_NUM - 1]) / SAMPLE_AVERAGE_NUM; 
    // Calculate the latest value of the discrete derivative
    lightSensorDerivative = (lightSensorVal - lightSensorValPrev) / (SINTERVAL); 
  }
  
  /**
   * initLightSensor.
   * Initializes the moving average for the light sensor value
   */
  private static void initLightSensor() {
    // Gets SAMPLE_AVERAGE_NUM samples from light sensor, stores them in the array of samples
    for (int i = 0; i < SAMPLE_AVERAGE_NUM; i++) { 
      lightSensorAvgVals[i] = fetchSample();
      // Increment the light sensor value mean by the most recent sample
      lightSensorVal += lightSensorAvgVals[i]; 
      sleepFor(SINTERVAL);
    }
    // Calculate the mean of values by dividing the sum by SAMPLE_AVERAGE_NUM
    lightSensorVal = lightSensorVal / SAMPLE_AVERAGE_NUM; 
  }

  /**
   * Returns a single sample from the sensor.
   */
  private static float fetchSample() {
    float[] colorValue = new float[colorSensor.sampleSize()];
    colorSensor.fetchSample(colorValue, 0);
    return colorValue[0];
  }
}