# Lab 2 - Odometry

_Read this entire document before doing anything._

**Code due: Immediately before your demo(s), Wednesday, September 23 at the latest**

**Report due: Friday, September 25, 23:59 EDT (Montréal time)**

This is the repository that contains the required files for the odometry lab.
For lab objectives, demo and report requirements, and submission instructions, see
detailed instructions on MyCourses.

## Design Process

Unlike the previous labs, you will need to implement more than one method. The increased
complexity will make it harder to track your changes if you don't use version control (ie, `git`).
Please contact a TA if you need help with this.

Since this a **design** course, we emphasize elements of the **design process** here. They will be
covered in more detail in the coming weeks.

1. The first thing you should do when starting a complex task to **understand the requirements**.
Go over the provided instructions, this README, and have a look at the starter code (but don't
change anything yet!) and make sure you and your lab partner know the lab objectives.

1. Work with your partner to come up with **high level designs** for different program components,
eg, calculating the odometry deltas. Keep these designs (eg, flowcharts, sketches, formula
derivations) since you will be asked to show them during the demo, and you should include the most
relevant ones in your report. Do this **before** writing any code.

1. **Implement** your designs in the given starter code. Both you and your partner must make meaningful
contributions to your submission. GitHub tracks these contributions automatically.

1. **Validate** your implementation to ensure it meets the all the given requirements.

1. If your implementation does not meet all requirements, go back to step 1 and **refine your design**.
It is unlikely that you will get everything right the first try, so don't be discouraged if you
repeat this cycle many times, that is expected.


## Implementation Details

We describe the starter code and a few implementation details below. You will need to complete
missing parts for all the files described below (including any tasks mentioned in the code but not
in this README). Since we cannot cover
all details here, please post any questions you have in the discussion board.

[`Main.java`](controllers/Lab2/ca/mcgill/ecse211/project/Main.java) is the main entry point for
your program. The main method sets up the program and starts the odometry and square driver
threads. The main simulation runs steps until Webots indicates that we're done
(eg, when we stop the simulation using the interface), then stops.

[`Odometer.java`](controllers/Lab2/ca/mcgill/ecse211/project/Odometer.java) keeps track of the
robot's position (_x_, _y_, _θ_) on the playing field. As you can imagine, this is very important
if you want to build a robot that can go to specific coordinates (which you will do in a future
lab). This class might seem large, but it follows a logical structure:
  * Class and instance variable declarations. You are allowed, but not required, to add variables
  of your own, but ask yourself first if they are truly necessary.
  * Private constructor and public getter. This is an implementation of the _singleton design pattern_,
  which ensures that there is only one odometer instance in your program. Set your robot starting point
  in the constructor. This point must be consistent with the location your robot starts at, as measured by
  the coordinate system described in the handout.
  * The `run()` method runs the odometry logic, which you need to complete, in an infinite loop.
  To ensure correct synchronization and to better organize the code, `run()` calls helper methods,
  some of which are detailed below.
  * `updateDeltaPosition()` updates the robot `deltaPosition` (_Δx_, _Δy_, _Δθ_) given the motor
  tacho counts. Pay close attention to the order of operations here! Make sure your calculations
  work "on paper" before attempting to implement them in code.
  * `updateOdometerValues()` increments the current values of the `x`, `y` and `theta` instance
  variables using `deltaPositions`, which was just calculated. `x` is already done but you need to do `y` and `theta`.
  Pay special attention to the latter (see comment in starter code).
  * `printPosition()` prints the odometer (_x_, _y_, _θ_) on the console. Ensure your output is
  easy to read, since this is what you will be demoing.
  * Setter methods. `setX()` is done for you. Complete the others in the same way, paying attention
  to thread synchronization.

[`Resources.java`](controllers/Lab2/ca/mcgill/ecse211/project/Resources.java) contains static 
resources (things that stay the same throughout the entire program execution), like constants and
hardware. Use these resources in other files by adding this line at the top (see given files for
examples):

```java
import static ca.mcgill.ecse211.project.Resources.*;
```

You will need to set some of the constants in this file based on your specific robot. You can
determine your robot dimensions using in Webots. Each node of your robot has a position, so you
can calculate your robot dimensions using that. You may need to tweak your values after testing.

[`SquareDriver.java`](controllers/Lab2/ca/mcgill/ecse211/project/SquareDriver.java) drives the
robot on the floor in a square trajectory. The length of the square is determined by the
`SQUARE_LENGTH` constant (which you should change during testing). The class structure is given
below:
  * `drive()` is a method that makes the robot drive in a square. It runs in a new thread to avoid
  blocking, ie, when this method is called, it will start a new thread and return control to the
  caller (where it was called from). Then, the actions in the thread (`stopMotors()` and so on) will
  run in the background. This method is given to you, but you must complete the helper methods
  it calls in order to make it work as described.
  * Helper methods. Implement these simple methods based on the given descriptions. All of them can
  be done in one or two lines (it is ok to use more, as long as your logic works). You may refer to
  the Lab 0 and Lab 1 starter code for guidance. For the ones that require a mathematical formula,
  make sure your derivation is correct before writing any code, to avoid unnecessary
  trial-and-error.

[`lab2.wbt`](worlds/lab2.wbt) is the Webots world file for Lab 2. It contains a DPM floor with
dimensions you can set by modifying the `dimension` line below:

```
DPM-Floor {
  dimension 5 5
  wallHeight 0.2
```

This is also where you will add DPM-Markers (see given world file for an example).
Reload the world to see your changes reflected in Webots.
