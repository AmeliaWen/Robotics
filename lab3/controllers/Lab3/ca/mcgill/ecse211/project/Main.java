package ca.mcgill.ecse211.project;

import static ca.mcgill.ecse211.project.Resources.*;
import static simlejos.ExecutionController.*;

import java.lang.Thread;


/**
 * Main class of the program.
 */
public class Main {
  
  /*
   * The number of threads used in the program (main, odometer), other than the one used to
   * perform physics steps.
   */
  public static final int NUMBER_OF_THREADS = 2;
  
  /** Flag to indicate which localization method to run. */
  public static final boolean IS_ULTRASONIC = false;
  /**How far the robot must reverse into the square to ensure light localization can occur.*/
  public static final double REVERSE_DISTANCE = 0.45; 
  
  /**
   * Main entry point.
   * 
   * @param args not used
   */
  public static void main(String[] args) {
    initialize();
    
    // Start the odometer thread
    new Thread(Odometer.getOdometer()).start();

    UltrasonicLocalizer.localize();
    
    pause();


    LightLocalizer.localize();
 
    Odometer.getOdometer().printPosition();
  }

  /**
   * Initializes the robot logic. It starts a new thread to perform physics steps regularly.
   */
  private static void initialize() {
    // Run a few physics steps to make sure everything is initialized and has settled properly
    for (int i = 0; i < 50; i++) {
      performPhysicsStep();
    }

    // We are going to start two threads, so the total number of parties is 2
    setNumberOfParties(NUMBER_OF_THREADS);
    
    // Does not count as a thread because it is only for physics steps
    new Thread(() -> {
      while (performPhysicsStep()) {
        sleepFor(PHYSICS_STEP_PERIOD);
      }
    }).start();
    
    leftMotor.setSpeed(FORWARD_SPEED);
    rightMotor.setSpeed(FORWARD_SPEED);
  }
  
  /**
   * Halts the robot for a while to allow pausing the simulation to evaluate ultrasonic
   * localization.
   */
  private static void pause() {
    System.out.println("Ultrasonic localization completed. Pause simulation now!");
    leftMotor.setSpeed(0);
    rightMotor.setSpeed(0);
    
    for (int i = 0; i < PAUSE_DURATION; i++) {
      waitUntilNextStep();
    }
    
    leftMotor.setSpeed(FORWARD_SPEED);
    rightMotor.setSpeed(FORWARD_SPEED);
  }

}
