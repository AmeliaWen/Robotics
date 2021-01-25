package ca.mcgill.ecse211.playingfield;

/**
 * Represents a coordinate point on the playing field grid.
 * 
 * @author Younes Boubekeur
 */
public class Point {
  /** The x coordinate in tile lengths. */
  public double x;

  /** The y coordinate in tile lengths. */
  public double y;

  /** Constructs a Point. The arguments are in tile lengths. */
  public Point(double x, double y) {
    this.x = x;
    this.y = y;
  }
  
  @Override
  public boolean equals(Object o) {
    if (!(o instanceof Point)) {
      return false;
    }
    
    Point other = (Point) o;
    return x == other.x && y == other.y;
  }
  
  @Override
  public final int hashCode() {
    return (int) (100 * x + y);
  }

  @Override
  public String toString() {
    return "(" + x + ", " + y + ")";
  }

}
