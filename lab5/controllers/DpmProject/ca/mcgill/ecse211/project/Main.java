package ca.mcgill.ecse211.project;

import static ca.mcgill.ecse211.project.Navigation.*;
import static ca.mcgill.ecse211.project.Resources.*;
import static simlejos.ExecutionController.*;

import ca.mcgill.ecse211.playingfield.Point;
import java.io.IOException;
import java.lang.Thread;
import java.nio.file.Files;
import java.util.Arrays;
import java.util.HashMap;
import java.util.List;
import java.util.Map;


/**
 * Main class of the program.
 */
public class Main {

  /**
   * The number of threads used in the program (main, odometer), other than 
   * the one used to perform physics steps. It is possible to do this lab 
   * with 2 threads, but you can add more.
   */
  public static final int NUMBER_OF_THREADS = 2;
  public static double[] averageTorques = new double[3];

  /**
   * The start and end points for the blocks, read from the vectors file 
   * path defined in Resources. Each vector entry has a number and a point array,
   *  where the first point is the vector head and the second is its tail. To access
   * these properties in your code, see the main method.
   * 
   * <p>Example: Block number 1 [(1, 3), (1, 5)]
   */
  public static Map<Integer, Point[]> vectors;

  /** Main entry point. */
  public static void main(String[] args) {
    initialize();

    // Start the odometer thread
    new Thread(odometer).start();

    
    UltrasonicLocalizer.localize();
    LightLocalizer.localize();
    System.out.println("localization done");

    int map = 0;
    // Iterate through the vectors and recognize which map it is
    for (var vector : vectors.entrySet()) {
      int number = vector.getKey();
      Point head = vector.getValue()[0];
      Point tail = vector.getValue()[1];
      System.out.println("Move block " + number + " from " + head + " to " + tail + ".");
      if (number == 1) {
        //Map selector based on first point
        switch ((int) head.x) {
          case 1:
            if (head.y == 7) {
              map = 4;
            } else {
              map = 5;
            }
            break;
          case 2:
            if (head.y == 3) {
              map = 3;
            } else {
              map = 5;
            }
            break;
          case 3:
            if (head.y == 6) {
              map = 1;
            } else if (head.y == 3) {
              map = 2;
            } else {
              map = 5;
            }
            break;
          default:
            map = 5;
        }

      }
    }
    //manually control whether to specify a map through input
    //or to detect which map it is 
    boolean force = true;
    if (force) {
      //Forced navigation.
      Navigation.map6(); //Do the map we want with no detection
    } else {
      //map navigation start
      switch (map) {
        case 1:
          System.out.println("Map 1 has been detected");
          map1();
          break;
        case 2:
          System.out.println("Map 2 has been detected");
          map2();
          break;
        case 3:
          System.out.println("Map 3 has been detected");
          map3();
          break;
        case 4:
          System.out.println("Map 4 has been detected");
          map4();
          break;
        default: // General navigation
      }

    }
    odometer.printPositionInTileLengths();

    /* Determine which block is heaviest */
    int maxIndex = 0;
    for (int i = 0; i < Navigation.averageTorqueValues.length; i++) {
      if (Navigation.averageTorqueValues[i] > Navigation.averageTorqueValues[maxIndex]) {
        maxIndex = i;
      }
    }

    maxIndex++; // Increment max index to represent block numbers
    System.out.println("The heaviest block is block " + maxIndex); // Print heaviest block

  }

  /**
   * Initializes the robot logic. It starts a new thread to perform physics steps regularly.
   */
  private static void initialize() {
    try {
      vectors = parseBlockVectors(Files.readAllLines(VECTORS_FILE));
    } catch (IOException e) {
      System.err.println("Could not open file: " + VECTORS_FILE);
      System.exit(-1);
    }

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

  /** Parses input lines into block vectors. */
  public static Map<Integer, Point[]> parseBlockVectors(List<String> lines) {
    var result = new HashMap<Integer, Point[]>();
    lines.forEach(line -> {
      if (!line.startsWith("#")) { // line is not a comment
        var n = Arrays.stream(line.split(" ")).map(Integer::parseInt).toArray(Integer[]::new);
        result.put(n[0], new Point[] {new Point(n[1], n[2]), new Point(n[3], n[4])});
      }
    });
    return result;
  }

}
