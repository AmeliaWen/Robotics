# Lab 5 - Navigation and Torque Measurement

_Read this entire document before doing anything._

This is the repository that contains the required files for the navigation and weight measurement lab.
For lab objectives, demo and report requirements, and submission instructions, see
detailed instructions on MyCourses.

## Milestones and due dates

Please consult MyCourses for all the milestones and their due dates.

## Updates and corrections

As with the previous labs, we will post any updates, corrections, and
clarifications to the starter code or these instructions on
[this page](https://mcgill-dpm.github.io/website/lab-notes).
Please check it regularly.

## Design Process

Since this a **design** course, once again we emphasize elements of the **design process** here.

1. The first thing you should do when starting a complex task to **understand the requirements**.
Go over the provided instructions, this README, and have a look at the starter code (but don't
change anything yet!) and make sure you and your teammates understand the lab objectives. If you have any questions, ask early!

1. Work with team to come up with **high level designs** for different program components,
eg, designing the `travelTo()` method. Keep these designs (eg, flowcharts, sketches, formula
derivations, etc) since you will be asked to show them during your meetings with the TA, and you should include the most
relevant ones in your report. Do this **before** writing any code.

   **The high-level design phase is extremely important: if your designs
are good, then implementation will be easy and will only take a few 
lines of code for each method. On the other hand, if you go straight to 
coding you will likely end up with a suboptimal solution at best!**

1. **Implement** your designs in the given starter code. All of you must make meaningful
contributions to your submission. GitHub tracks these contributions automatically.

1. **Validate** your implementation to ensure it meets the all the given requirements.
We included _unit tests_ to help you with this (more on this below).
You may write your unit tests before your code.

1. If your implementation does not meet all requirements, go back to step 1 and **refine your design**.
It is unlikely that you will get everything right the first try, so don't be discouraged if you
repeat this cycle many times, as that is expected.


## Prerequisites

Make sure you have set up your system as described in the previous labs
and the Getting Started Guide.

- [ ] Java 11 or higher installed and accessible from the terminal
(command line). Run `java`, `javac`, and `jar` to verify this.

- [ ] LeoCAD and Webots. You must be able to build and run the simulation
from your own machine, without relying on another team member.

- [ ] Eclipse. Make sure you use the Eclipse preferences file,
[CheckStyle](https://mcgill-dpm.github.io/website/EclipseCheckstyleSpotbugs),
and set the `WEBOTS_HOME` [classpath variable](https://mcgill-dpm.github.io/website/EclipseClasspathVariables)
if you have not already done so in Lab 3.

- [ ] GNU Awk (optional). This is required for generating HTML test reports 
with `make test`. More information is available [here](https://mcgill-dpm.github.io/website/JUnit).


## Implementation Details

We describe the starter code and a few implementation details below. You will need to complete
missing parts for all the files described below (including any tasks not explicity mentioned in this README). Since we cannot cover
all details here, please post any questions you have in the discussion board.

[`Main.java`](controllers/DpmProject/ca/mcgill/ecse211/project/Main.java) is the main entry point for
your program. The main method sets up the program and starts the needed
threads. One of the things you need to determine in this lab is how many
threads do you need, and what they should each do. Think carefully before 
spawning new threads.

Before starting the simulation, the main class first reads the
[block vector file](controllers/DpmProject/vectors.txt), specified in 
[`Resources.java`](controllers/DpmProject/ca/mcgill/ecse211/project/Resources.java), to the `vectors` variable, which maps each block
number to a pair of `Point`s representing the head and tail of the vector.
You need to move each block from the head to the tail. The starter code
includes an example of accessing the number, head and tail values in the 
main method. Use that example, along with the methods in other classes, 
to realize the requirements for this lab. Don't forget about measuring
the block weights while pushing them.

[`Navigation.java`](controllers/DpmProject/ca/mcgill/ecse211/project/Navigation.java) is a class that contains static methods used to navigate
around the playing field. Navigation must **not** be a thread, although
it can interact with other threads or create new ones. Implement the
following methods:

  * `travelTo()` makes the robot travels to the given destination. Think carefully about the design of this method in particular, including edge cases, like a block that prevents you from traveling straight to your destination. Also think about how this method will interact with the light localization you implemented in the previous lab, if applicable.
  * `turnTo()` turns the robot with a minimal angle towards the given input angle in degrees, no matter what its current orientation is. This method is different from `turnBy()`, which ignores the odometer's current angle. A **minimal angle** is one that does not exceed π radians (180°). For example if the robot is pointing towards 45° and
  `turnTo(135)` is called, the robot must turn 90° clockwise. It must not
  turn 270° counterclockwise, as that would be the maximal angle.

    <div  style="text-align:center;"><img src="https://mcgill-dpm.github.io/website/resources/labs/min-angle.png" width="292" height="223"><br><br></div>

  * `getDestinationAngle()` computes and returns the robot's destination
  angle, given its current location and the destination it should point to. For example, if the robot is at (1, 1) and the destination is
  (0, 2), then the function should return 315°.
  * `minimalAngle()` returns the signed minimal angle in degrees from initial angle to destination angle, both in degrees. "Signed" means that
  the return value could be positive or negative, with the sign determining the direction of the angle (DPM conventions use positive for clockwise and negative for counterclockwise). A minimal angle example is given above.
  * `distanceBetween()` returns the distance between the two points in tile lengths (feet).

  * Helper methods. These could be from the previous labs, eg `turnBy()`,
  or new helper methods you implement to help with the methods above or
  to perform other navigation-related tasks. Make sure your methods are
  well documented with Javadocs, which always start with `/**` (slash
  followed by **two** stars).

    **Hint:** Some of these methods are very simple to implement if the 
  others are already done, so think about your implementation order carefully.

[`Odometer.java`](controllers/DpmProject/ca/mcgill/ecse211/project/Odometer.java) keeps track of the
robot's position (_x_, _y_, _θ_) on the playing field. Remember that this
position is metric, whereas the tiles are in feet (this constraint is
from the real world floors, so we cannot change it). The odometer provided
in the starter code is very similar to the one provided in the previous
lab, with the addition of one convenience method:
  * `printPositionInTileLengths()` prints the odometer (_x_, _y_, _θ_) on the console using the tile coordinate system, to make it easier for you to know where your robot thinks it is.

[`Resources.java`](controllers/DpmProject/ca/mcgill/ecse211/project/Resources.java) contains static 
resources (things that stay the same throughout the entire program execution), like constants and
hardware. Use these resources in other files by adding this line at the top (see given files for
examples):

```java
import static ca.mcgill.ecse211.project.Resources.*;
```

Once again, you will need to set some of the constants in this file based on your specific robot. You can
determine your robot dimensions using in Webots. Each node of your robot has a position, so you
can calculate your robot dimensions using that. You may need to tweak your values after testing.

[`vectors.txt`](controllers/DpmProject/vectors.txt) is a file that contains the block vectors in this format:

```
# Format: block_id x1 y1 x2 y2
1 1 3 1 5
```

In other words, the above line tells us to move block number 1 from
(1, 3) to (1, 5). Remember to measure the block's weight while moving it!
In the same folder, you can find four other files, `vectors1.txt`,
`vectors2.txt`, and so on, which are identical to the four maps provided
in the handout. You can select these maps in your code either by:

* Changing the file path in the `Resources` class, or
* Copy/pasting the desired file content into `vectors.txt`.

The advantage of the second option is that you do not need to rebuild 
your code, since you only changed a text file and not the Java files.

<div style="text-align:center;"><img src="https://mcgill-dpm.github.io/website/resources/labs/example_block_vectors.png" width="376" height="376"><br><br></div>

[`lab5.wbt`](worlds/lab5.wbt) is the Webots world file for Lab 5.
Note that in this lab the floor has two colors, so make sure your
controller can detect lines in both regions under different conditions.
The competition arena will include even more colors! :rainbow:

The world also includes the blocks that you need to move and measure.
**Make sure the blocks in the world are consistent with the vectors file!**

## Using JUnit to validate your logic

A _unit test_ is a piece of code written to test a _unit_ of code that you
write. A unit is most often a function or method, but could also be a class.
Manual testing can be time-consuming and error prone, so automating the 
testing process can help you fix bugs faster and write better code.

The most popular unit testing framework for Java is
[JUnit](https://junit.org/junit5/docs/current/user-guide).
We provide DPM-specific instructions [here](https://mcgill-dpm.github.io/website/JUnit).

In addition to helping you in the design process, writing good unit tests is worth bonus marks in this lab.

## Using Javadoc to document your implementation

Since the beginning of the course, you have been asked to document your 
classes, class members, and methods using Javadoc comments (which start with
`/**`). The `javadoc` tool converts these comments into easy-to-read HTML 
documentation, which looks like [this](https://junit.org/junit5/docs/current/api/).
It is very easy to generate this output from your code in Eclipse by 
selecting Project > Generate Javadoc and following the prompts. The root of 
the Javadocs will be `index.html`.

You must submit a Javadoc API once you get to the project, so it is good to 
practice doing this during Lab 5.
