package ca.mcgill.ecse211.test;

import static ca.mcgill.ecse211.project.Navigation.*;
import static ca.mcgill.ecse211.project.Resources.*;
import static org.junit.jupiter.api.Assertions.*;

import org.junit.jupiter.api.Test;

import ca.mcgill.ecse211.playingfield.Point;

/**
 * Tests the Navigation class. This test runs in Eclipse (right-click > Run as > Unit test) and
 * on the command line, not in Webots!
 * 
 * @author Younes Boubekeur
 */
public class TestNavigation {
  
  /** Tolerate up to this amount of error due to double imprecision. */
  private static final double ERROR_MARGIN = 0.01;
  
  @Test void testMinimalAngle() {
    // Going from 45° to 135° means turning by +90°
    assertEquals(90, minimalAngle(45, 135), ERROR_MARGIN);
    
    // Going from 185° to 175° means turning by -10°
    assertEquals(-10, minimalAngle(185, 175), ERROR_MARGIN);
    assertEquals(0, minimalAngle(0, 0), ERROR_MARGIN);
    assertEquals(-80, minimalAngle(185, 105), ERROR_MARGIN);
    assertEquals(-180, minimalAngle(185, 5), ERROR_MARGIN);
    
    // TODO Add more test cases here. Don't forget about edge cases!
  }
  @Test void testDestinationAngle() {
	  Point x = new Point(1,1);
	  Point y = new Point(0, 2);
	 
	    assertEquals(315, getDestinationAngle(x,y), ERROR_MARGIN);
	    Point x1 = new Point(1,1);
		 Point y1 = new Point(1, 2);
		    
		    assertEquals(270, getDestinationAngle(x1,y1), ERROR_MARGIN);
		    Point x2 = new Point(1,1);
			  Point y2 = new Point(1, 0);
			    
			    assertEquals(90, getDestinationAngle(x2,y2), ERROR_MARGIN);
			    Point x3 = new Point(1,1);
				  Point y3 = new Point(0, 1);
				    
				    assertEquals(180, getDestinationAngle(x3,y3), ERROR_MARGIN);
				    Point x4 = new Point(1,1);
					  Point y4 = new Point(2, 1);
					    
					    assertEquals(0, getDestinationAngle(x4,y4), ERROR_MARGIN);
				    
	    
	  }
  
  // TODO Think about testing your other Navigation functions here
  
  // We can add helper methods below to be used in the tests above

}
