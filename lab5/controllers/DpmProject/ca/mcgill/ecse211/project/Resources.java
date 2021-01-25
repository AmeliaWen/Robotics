package ca.mcgill.ecse211.project;

import java.nio.file.Path;
import java.nio.file.Paths;
import simlejos.hardware.motor.Motor;
import simlejos.hardware.port.SensorPort;
import simlejos.hardware.sensor.EV3ColorSensor;
import simlejos.hardware.sensor.EV3UltrasonicSensor;
import simlejos.robotics.RegulatedMotor;
import simlejos.robotics.SampleProvider;

/**
 * Class for static resources (things that stay the same throughout the entire program execution),
 * like constants and hardware.
 * <br><br>
 * Use these resources in other files by adding this line at the top (see examples):<br><br>
 * 
 * {@code import static ca.mcgill.ecse211.project.Resources.*;}
 */
public class Resources {
  
  /** The time between physics steps in milliseconds. */
  public static final int PHYSICS_STEP_PERIOD = 500; // ms
  
  /** The path of the input vector file, relative to controllers/DpmProject. */
  public static final Path VECTORS_FILE = Paths.get("vectors1.txt");
  
  // TODO Adjust the following parameters based on your robot
  
  /** The maximum distance detected by the ultrasonic sensor, in cm. */
  public static final int MAX_SENSOR_DIST = 50;
  
  /** The limit of invalid samples that we read from the US sensor before assuming no obstacle. */
  public static final int INVALID_SAMPLE_LIMIT = 20;
  
  /** The wheel radius in meters. */
  public static final double WHEEL_RAD = 0.021;
  
  /** The robot width in meters. */
  public static final double BASE_WIDTH = 0.0972;
  
  /** The distance between the color sensors and the wheels in meters. */
  public static final double COLOR_SENSOR_TO_WHEEL_DIST = 0;
  
  /** The speed at which the robot moves forward in degrees per second. */
  public static final int FORWARD_SPEED = 250;
  
  /** The speed at which the robot rotates in degrees per second. */
  public static final int ROTATE_SPEED = 250;
  
  /** The motor acceleration in degrees per second squared. */
  public static final int ACCELERATION = 2000;
  
  /** The tile size in meters. Note that 0.3048 m = 1 ft. */
  public static final double TILE_SIZE = 0.3048;
  
  /** The cube side length in meters.*/
  public static final double CUBE_SIDE = 0.15;
  
  /** How many samples of color sensor data to take average of. */
  public static final int LIGHT_FILTER_SIZE = 3;
  
  /** The poll sleep time, in milliseconds. */
  public static final int POLL_SLEEP_TIME = 50;
  
  /** The odometer. */
  public static Odometer odometer = Odometer.getOdometer();
  
  // Hardware resources

  /** The left motor. */
  public static final RegulatedMotor leftMotor = Motor.A;
  
  /** The right motor. */
  public static final RegulatedMotor rightMotor = Motor.D;
  
  /** The ultrasonic sensor. */
  public static final EV3UltrasonicSensor usSensor = new EV3UltrasonicSensor(SensorPort.S1);
  
  /** The left color sensor. */
  public static final SampleProvider colorSensor = new EV3ColorSensor(SensorPort.S2).getRedMode();
  
  public static final SampleProvider colorSensor2 = new EV3ColorSensor(SensorPort.S3).getRedMode();

  
}
