package ca.mcgill.ecse211.project;

import static ca.mcgill.ecse211.project.Navigation.*;
import static ca.mcgill.ecse211.project.Resources.*;
import static simlejos.ExecutionController.sleepFor;

/**
* this class contains the necessary methods for the robot to navigate to a grid intersection 
* with the help of one or two color sensors.
*/
public class LightLocalizer {
  /**
  * this method navigates the robot to (1,1) assuming that the orientation is already zero degree.
  */
  public static void localize() {
    //Rotation by 90 degrees changes the translation coordinate of the robot
    //Through testing, we found that the robot moves ahead by 0.055 to to the left/right
    // by 0.055 when the robot rotates clock-wise/counterclockwise
    double offset = 0.061;
    goToNextLine();
    Navigation.moveStraightFor(offset);
    Navigation.turnBy(90.0);
    goToNextLine();
    Navigation.moveStraightFor(offset);
    offset = -0.085;
    Navigation.moveStraightFor(offset); //addresses the change in coordinate due to rotation
    Navigation.turnBy(-90.0);
    offset = -0.092;
    Navigation.moveStraightFor(offset); 
    //odometer.setTheta(0);
    odometer.setXyt(TILE_SIZE, TILE_SIZE, 0);
   // odometer.set
  }
 
  /**
   * This method is used to move the robot to the next horizontal line and stop.
   * By the end of this method, the center of the robot will be right on top of the horizontal line
   * Assumes robot to be perpendicular to the line at first.
   */
  public static void goToNextLine() {
  
    boolean leftStop = false;
    boolean rightStop = false;
    
    float[] colorData = new float[LIGHT_FILTER_SIZE];
    float present = 0; //last average intensity value 
    float past = present; //current average intensity value
    float[] colorData2 = new float[LIGHT_FILTER_SIZE];
    float present2 = 0; //last average intensity value for second color sensor
    float past2 = present2; //current average intensity value
    for (int i = 0; i < 50; i++) { //lets the sensor data "settle down"
      colorSensor.fetchSample(colorData, 0);
      colorSensor2.fetchSample(colorData2, 0);
    }
    Navigation.startMotors();
    
    //repeatedly average data, then compare to the previous recorded value
    //A large decrease signal light to dark transition
    while (!(leftStop && rightStop)) {
      //fill up the colorSensor array
      for (int i = 0; i < LIGHT_FILTER_SIZE; i++) {
        colorSensor.fetchSample(colorData, i);
        present += colorData[i];
        colorSensor2.fetchSample(colorData2, i);
        present2 += colorData2[i];
        sleepFor(POLL_SLEEP_TIME);
      }
      //creates average
      present /= LIGHT_FILTER_SIZE;
      present2 /= LIGHT_FILTER_SIZE;
     // System.out.println(present);
      //if past value has been initialized and there is a sudden decrease in average light intensity
      if (past != 0 && present + 15 < past) {
        rightMotor.stop();
        rightStop = true;
        //break;
      }
      if (past2 != 0 && present2 + 15 < past2) {
        leftMotor.stop();
        leftStop = true;
         
      }
      past = present; //updates variable for past intensity
      past2 = present2;
    }
    Navigation.setBothSpeed(FORWARD_SPEED);
    //moves ahead so that the center of robot aligns with horizontal line
    
    
  }
  
}
