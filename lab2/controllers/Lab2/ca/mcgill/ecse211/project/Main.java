package ca.mcgill.ecse211.project;

import static ca.mcgill.ecse211.project.Resources.*;
import static simlejos.ExecutionController.*;

import java.lang.Thread;


/**
 * Main class of the program.
 */
public class Main {
  
  /** The number of threads used in the program (main, odometer, square driver). */
  public static final int NUMBER_OF_THREADS = 3;
  
  /**
   * Main entry point.
   * 
   * @param args not used
   */
  public static void main(String[] args) {
    // Run a few physics steps to make sure everything is initialized and has settled properly
    for (int i = 0; i < 50; i++) {
      performPhysicsStep();
    }
    
    // We are going to start two threads, so the total number of parties is 3
    setNumberOfParties(NUMBER_OF_THREADS);
    (new Thread(Odometer.getOdometer())).start();
    // Start the square driver, which internally starts a thread
    SquareDriver.drive();
    // Main simulation loop, run steps until WeBots indicates that we're done
    while (performPhysicsStep()) {
      // do nothing
    }

    System.exit(0);
  }

}
