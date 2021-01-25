package ca.mcgill.ecse211.project;

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
  
  /** The duration of the pause between ultrasonic and light localization. */
  public static final int PAUSE_DURATION = 10000; // performPhysicsStep() calls
  
  // TODO Adjust the following parameters based on your robot
  
  /** The wheel radius in meters. */
  public static final double WHEEL_RAD = 0.021;
  
  /** The robot width in meters. */
  public static final double BASE_WIDTH = 0.144;
  
  /** The distance from the robot centre to the light sensor in meters. */
  public static final double LIGHT_SENSOR_DISTANCE = 0.13;
  
  /** The speed at which the robot moves forward in degrees per second. */
  public static final int FORWARD_SPEED = 250;
  
  /** The speed at which the robot rotates in degrees per second. */
  public static final int ROTATE_SPEED = 150;
  
  /** The motor acceleration in degrees per second squared. */
  public static final int ACCELERATION = 3000;
  
  /** The tile size in meters. Note that 0.3048 m = 1 ft. */
  public static final double TILE_SIZE = 0.3048;
  
  /** The maximum distance detected by the ultrasonic sensor, in cm. */
  public static final int MAX_SENSOR_DIST = 255;
  
  /** The limit of invalid samples that we read from the US sensor before assuming no obstacle. */
  public static final int INVALID_SAMPLE_LIMIT = 20;
  
  /** The odometer. */
  public static Odometer odometer = Odometer.getOdometer();
  
  /** The left motor. */
  public static final RegulatedMotor leftMotor = Motor.A;
  
  /** The right motor. */
  public static final RegulatedMotor rightMotor = Motor.D;
  
  /** The ultrasonic sensor. */
  public static final EV3UltrasonicSensor usSensor = new EV3UltrasonicSensor(SensorPort.S1);
  
  /** The color sensor sample provider. */
  public static final SampleProvider colorSensor = new EV3ColorSensor(SensorPort.S2).getRedMode();
  
}
