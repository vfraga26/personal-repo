
// ~ MARIST BASE ROBOT PROGRAM ANALYSIS V1
// ~
// ~ Analyzed by Vanessa Fraga, Class of 2026.
// ~ Hello! This is an analysis of the Marist Base Code for FTC Robots.
// ~ Any comments marked with "~" is my commentary and any other comments
// ~ are from the original program.

// ~ This is a package from FTC to program FTC Robots. Packages are used to group
// ~ and access necessary classes so that they can be imported to the program.
// ~ We will call this package "teamcode".

package org.firstinspires.ftc.teamcode;

// ~ These imports are classes from teamcode. These classes create the scaffolding
// ~ that allow you to build the Base Code. Some of these classes are not necessary
// ~ for our purposes, so they will be grouped together and noted.

// ~ These classes are necessary to program the robot.

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

// ~ For documentation purposes, there will be a brief explaination of these classes.
// ~ These classes are add-ons that build the robot's capabilites of reading color,
// ~ sensing motion and/or objects, turning on/off a light, field-oriented driving
// ~ (which will be explained later on), and find the angle of the robot's position.

// ~ These classes are optional but are kept in the program for possible future use.

import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.SwitchableLight;
import android.graphics.Color;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;


// ~ The below are credits to the original programmers and revisions done to
// ~ this program.

/*
 * Created by michaudc on 10/8/2017.
 * Updated by michaudc on 29 July 2021
 * Additional code by Allesio Toniolo July 2021
 * Based on HardwarePushbot Code from the FTCRobotController resources
 * 
 * Revision for 2022 Season V1 on 19 Aug 22 by michaudc
 *
 * Revision for 2023 Season V1 on 24 July 22 by michaudc
 * Added Java Karel style method calls for move, turnLeft, turnRight, turn
 * Overloaded Methods with (distance, speed) and (distance) formats
 * 
 * Revision for 2024-2025 Season V1 05 Jun 24 by Cole S, Russell M, and michaudc
 * Added methods for field centric drive in Base robot
 * Revision for 2024 Updated getBrightness() to return value from HSV
 */

// ~ This section is very important to understand. This helps connect the Base
// ~ Code to the Driver Station Configuration. This also introduces Java
// ~ Naming and Configuration Naming.
// ~ The JAVA NAME of an electronic is used to reference the electronic within the
// ~ program. We commonly write a Java Name in the camelCase format.
// ~ The CONFIGURATION NAME of an electronic is used to name the electronic in the
// ~ Driver Station Configuration. It is recommended that the Configuration Name
// ~ is similar or the same as the Java Name for clarity. 
// ~ We also note what type of electronic is associated with each Java and 
// ~ Configuration Name. This is a type of Configuration Mapping.

/*
 * This class models the physical structure of the robot with instances
 * of motors, servos, and sensors.
 *
 * The following are name assignments to be configured 
 * on the Driver Station.
 *
 * [Electronic Type]    [Java Name]    [Configuration Name]
 *  Motor channel:      leftFront:        "leftfront"
 *  Motor channel:      rightFront:       "rightfront"
 *  Motor channel:      leftRear:         "leftrear"
 *  Motor channel:      rightRear:        "rightrear"
 *  Motor channel:      leftArm:          "leftarm"
 *  Motor channel:      rightArm:         "rightarm"
 *  Servo Channel:      leftHand:         "lefthand"
 *  Servo Channel:      rightHand:        "righthand"
 *  Touch Sensor:       touch             "touch"   ** Digital 1 in Configuation **
 *  Color Sensor:       colorSensor       "colorSensor"
 */

// ~ Now, this is where we begin seeing the bulk of the code. This line 
// ~  " public class MaristBaseRobot2024_Quad { "
// ~ creates the class that holds all the methods that we will use in TeleOp 
// ~ and Autonomous. !IMPORTANT! The partner bracket to this line is at the
// ~ end of this program. PLEASE DO NOT DELETE IT, BAD THINGS HAPPEN. There 
// ~ will be a note above the partner bracket to note of it's importance.

public class MaristBaseRobot2024_Quad {

// ~ These are the attributes of our class. This section of our attributes 
// ~ are for our Motors and Servos. Note that these are public attributes 
// ~ and not private. This is because we want to use these attributes across 
// ~ all related programs. We also make these attributes equal to "null"
// ~ so they are reserved and kept within scope. 
// ~ We structure an attribute with the following,
// ~  " public [Electronic] [Java Name] = null; "

      /* Public Motors and Servos */

    public DcMotor leftFront   = null;
    public DcMotor rightFront  = null;
    public DcMotor leftRear    = null;
    public DcMotor rightRear   = null;
    public DcMotor rightArm    = null;
    public DcMotor leftArm     = null;
    public Servo rightHand     = null;
    public Servo leftHand      = null;

// ~ It is important to understand the unique use these two lines below
// ~ have relating to driving orientation.
// ~ There are two types of orientations that can be used to move the 
// ~ robot, FIELD ORIENTATION and ROBOT ORIENTATION. 
// ~ ROBOT ORIENTATION is when the cardinal directions are applied to the
// ~ physical robot.
// ~ FIELD ORIENTATION is when the cardinal directions are applied to the
// ~ robot field.
// ~ IMU stands for "Inertial Measurement Unit" and acts like the Robot's 
// ~ nervous system. The variable "offset" is also used for the same reason.
// ~ These two lines are used to program the driving orientation to be Field
// ~ Oriented. We do not recommend doing this. Robot Drivers are more adapted
// ~ to drive using Robot Orientation and will use their own senses to drive
// ~ the robot. To save time, please stick to Robot Orientation.

    public IMU imu;
    public double offset = 0;  //In degrees.

// ~ This section of our attributes are for our Sensors. 

    		 /* Public Sensors */
    public DigitalChannel touch              = null;
    public NormalizedColorSensor colorSensor = null;

// ~ The following section are our constants and limiters. Any variable that
// ~ is uppercase is a constant, and will never change unless altered in its 
// ~ own final variable. 
// ~ The first three final variables are for controlling the power of any
// ~ motors in an arm. It is not recommended to change these for the safety of
// ~ the motors (from overheating) and students (from physical injury).

    // Constants for Arm and Servo Operation
    public static final double MID_SERVO        =  0.5;
    public static final double ARM_UP_POWER     =  2.00;
    public static final double ARM_DOWN_POWER   = -2.00;

// ~ These following attributes, to note, are not public attributes. Since we
// ~ do not need to alter or use these attributes outside of the Base Code, they
// ~ are private and kept within this program. These attributes are for the
// ~ Encoders and their purpose is to help program for Autonomous and will be
// ~ explained in a different program.

    // For Encoder Functions
    private double     COUNTS_PER_MOTOR_REV          = 1440 ;    // eg: TETRIX Motor Encoder
    private final double     DRIVE_GEAR_REDUCTION    = 1.0 ;     // This is < 1.0 if geared UP
    private double     WHEEL_DIAMETER_INCHES         = 4.0 ;     // For figuring circumference
    private double     COUNTS_PER_INCH               = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.1415);
    private double COUNTS_PER_DEGREE                 = COUNTS_PER_MOTOR_REV / 360;

// ~ These two attributes are important for limiting the speed of the robot.
// ~ While testing your TeleOp and Auto, it is recommended to not change these
// ~ variables for safety reasons (see two sections back).
// ~ # Mr. Michaud Recommends! #
// ~ "When you are done testing your TeleOp and Autonomous, change the limit
// ~  for both variables to 1.0. This allows you to maximize the speed of
// ~  your robot for competition!"
// ~ (Note: It is prohibited from changing this limit to anything above 1.0 as
// ~  there will be overheating in the motors.)

    private double     DRIVE_SPEED                   = 0.6;   // Speed Limit for driving.
    private double     TURN_SPEED                    = 0.5;   // Speed Limit for turning.

// ~ This is the an important part of the Base Code. This is where we tie 
// ~ software to hardware. We create a HardwareMap object called "hwMap"
// ~ to retreive the Configuration Names from the Driver Station
// ~ and relate it to the Java Names. This will be done in a later section.
// ~ The variable "ElapsedTime" is not relevant to our purposes, so please 
// ~ disregard.
// ~ Also take note of the "Local OpMode" being mentioned. Operational Mode
// ~ (or just OpMode) is a class that adds a program to the Driver Station
// ~ so we can access and initialize programs. This will be used in the program's
// ~ methods to confirm if OpMode was initialized so it won't accidently 
// ~ execute 
 
    // Local OpMode members
    HardwareMap hwMap = null;
    private ElapsedTime period = new ElapsedTime();

// ~ This is our constructor for our class of the same name. The reason why 
// ~ it is blank is because we already have a method that acts in place of
// ~ the constructor, the init method in the next section.

    // Constructor - leave this blank for now.
    public MaristBaseRobot2024_Quad () {

    }

// ~ The below method is the initialization method, or the init method. This
// ~ method takes the hwMap (which is the configuration in the Driver Station)
// ~ and initializes (connects) it to the program. This method acts like the
// ~ constructor. 

    /* Initialize standard Hardware interfaces */

    public void init(HardwareMap ahwMap) {

// ~ Note: Mr. Michaud acknowledges that the reference is redundant as it can
// ~ can be removed and the parameter rewritten as "HardwareMap hwMap" and it
// ~ will still work. This reference is saved for learning purposes but it is
// ~ recommended that this is removed and the parameter rewritten for clarity.

	// Save reference to Hardware map.
        hwMap = ahwMap;

// ~ This is where the initialization begins. we change the attributes value
// ~ from "null" to the electronic's configuration name. The format is as
// ~ followed,
// ~ " [Java Name] = hwMap.[Electronic].get("[Configuration Name]"); "
// ~ This essentially says,
// ~ "Look, this variable? Go in the hwMap, go look at this specific group
// ~  of electronics, find (get) this string, and assign it to our variable."
// ~ This will also be done to other electronics and will be noted.

        // Define and Initialize Motors.  Assign names that match the setup on the Driver Station.
        leftFront    = hwMap.dcMotor.get("leftfront");
        rightFront   = hwMap.dcMotor.get("rightfront");
        leftRear     = hwMap.dcMotor.get("leftrear");
        rightRear    = hwMap.dcMotor.get("rightrear");
        leftArm      = hwMap.dcMotor.get("leftarm");
        rightArm     = hwMap.dcMotor.get("rightarm");

// Usually, FORWARD is clockwise and REVERSE is counter-clockwise. It is important to distiguish this.

// ~ This section is assigning a Forward or Reverse direction to each of the
// ~ motors. To understand the need for this differientation, we must look at 
// ~ a physical motor. Point the motor's shaft towards you. 
// ~ When the shaft moves in a clockwise motion, it is moving in the FORWARD
// ~ direction. When the shaft moves in a counter-clockwise motion, it is 
// ~ moving in the REVERSE direction. 
// ~ You would have also learned this in your manual along with Fleming's
// ~ Left-Hand Rule for Motors.

        leftFront.setDirection(DcMotor.Direction.FORWARD);  // Set to REVERSE if using AndyMark motors
        rightFront.setDirection(DcMotor.Direction.REVERSE); // Set to FORWARD if using AndyMark motors
        leftRear.setDirection(DcMotor.Direction.FORWARD);   // Set to REVERSE if using AndyMark motors
        rightRear.setDirection(DcMotor.Direction.REVERSE);  // Set to FORWARD if using AndyMark motors
        leftArm.setDirection(DcMotor.Direction.FORWARD);
        rightArm.setDirection(DcMotor.Direction.FORWARD);

// ~ We set the power to zero to prevent movement while initializing the 
// ~ Robot in the Driver Station. 

        // Set all motors to zero power
        leftFront.setPower(0);
        rightFront.setPower(0);
        leftRear.setPower(0);
        rightRear.setPower(0);

// ~ We set the robot to run without encoders because we assume that TeleOp
// ~ is being run. 

        // Set all motors to run without encoders. This is by default.
        // May want to use RUN_USING_ENCODERS if encoders are installed.
        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

// ~ We initialize the servos next. It is done in a similar way as the
// ~ motors, but the electronic is now "servo" instead of "dcMotor".

        // Define and initialize ALL installed servos.
        leftHand = hwMap.servo.get("lefthand");
        rightHand = hwMap.servo.get("righthand");

// ~ Sets the servo to an automatic position. It is recommended not to set
// ~ a position in the base code due to the FTC rules not allowing movement
// ~ during initializing.

        //leftHand.setPosition(MID_SERVO);
        //rightHand.setPosition(MID_SERVO);

// ~ The following lines are not relevant to you if you are not planning on
// ~ using them. For documentation purposes, they simply initalize the add-ons
// ~ that were listed previously.

        // Define and Initialize the Touch and Color Sensors.
        touch = hwMap.get(DigitalChannel.class, "touch");
        colorSensor = hwMap.get(NormalizedColorSensor.class, "colorSensor");
        
        // Initializing the IMU Sensor for Field Centric Driving.
        imu = hwMap.get(IMU.class, "imu");

        // Define for Orientation of REV Control Hub. This is for Field-Orientated Drive.
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.BACKWARD;
        RevHubOrientationOnRobot.UsbFacingDirection  usbDirection  = RevHubOrientationOnRobot.UsbFacingDirection.UP;

        // Initialize the IMU. The initalizing of the nervous system.
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);
        imu.initialize(new IMU.Parameters(orientationOnRobot));
        
	// Initialize the input from the Touch Sensor.
        touch.setMode(DigitalChannel.Mode.INPUT);

    }

// ~ waitForTick is not important for our purposes, but for documentation
// ~ reasons, will be left in. There is already an analysis written by the
// ~ original creators, so there is no explaination needed from me.
	
    /*
     * waitForTick implements a periodic delay. However, this acts like a metronome with a regular
     * periodic tick.  This is used to compensate for varying processing times for each cycle.
     * The function looks at the elapsed cycle time, and sleeps for the remaining time interval.
     *
     * @param periodMs		Length of wait cycle in mSec.
     */

    public void waitForTick(long periodMs) {

        long  remaining = periodMs - (long)period.milliseconds();

        // sleep for the remaining portion of the regular cycle period.
        if (remaining > 0) {
            try {
                Thread.sleep(remaining);
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
            }
        }

        // Reset the cycle clock for the next pass.
        period.reset();
    }

// ~ Here is where we begin to see the beginning of the methods that are
// ~ used for the robot's autonomous. There are five original methods that
// ~ can be used for an autonomous program,
// ~  1). driveStraightInches() , move x amount of inches foward/backward.
// ~  2). pointTurnDegrees() , turn x amount of degrees left/right.
// ~  3). strafeInches() , strafe x amount of inches left/right.
// ~  4). leftArmMotorDeg() , rise/lower the arm by x degrees.
// ~  5). (Optional) rightArmMotorDeg() , similar to leftArmMotorDeg() but
// ~      this method is for your second arm (if added).
// ~ We will go over all five methods and explain how they were built and
// ~ how they are used.
	
    // Additional Functions (i.e. Methods) to control Servos and motors

    public void driveStraightInches(double speed, double inches, double timeoutS) {
	
// ~ The above signature asks for three variables from the programmer.
// ~  1). "double speed" is to decide how fast the robot should drive.
// ~  2). "double inches" is to tell the robot how far it should drive.
// ~  3). "double timeoutS" is to stop the program after a certain amount
// ~      of time.
// ~ The next four int variables that are created will be used to calculate
// ~ the distance (and by extension the amount of rotation of the motors) that
// ~ the robot will travel when it the method finishes executing. 
	    
	// Creates new variables for the Scope of this method.
        int newLeftFrontTarget;
        int newRightFrontTarget;
        int newLeftRearTarget;
        int newRightRearTarget;

        // Reverse inches. Goofy and Hacky. Essential to configure.
        inches = inches * -1 * (12.0/39.0);

// ~ This section resets the encoders for all the motors and re-initializes
// ~ them. We do this so the robot receives the original information for the
// ~ motors in case we did any previous altercations to the encoder variables.
// ~ It also helps keep all the motors synchronized so that the robot uses
// ~ our code correctly.
	    
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

// ~ This is a safely measure in case the speed variable is set to an unsafe
// ~ level. In case the speed variable is above the limit, this statement
// ~ sets the speed variable to the speed limit.

        // Set to Limit of DRIVE_SPEED
        if (Math.abs(speed) > DRIVE_SPEED) {
            speed = DRIVE_SPEED; //
        }


        // Ensure that the opmode is still active
        //if (opModeIsActive()) {
        if (true) {       // Swapped out to include in MaristBaseRobot

            // Determine new target position, and pass to motor controller. Notice the casting.
            newLeftFrontTarget = leftFront.getCurrentPosition() + (int)(inches * COUNTS_PER_INCH);
            newRightFrontTarget = rightFront.getCurrentPosition() + (int)(inches * COUNTS_PER_INCH);
            newLeftRearTarget = leftRear.getCurrentPosition() + (int)(inches * COUNTS_PER_INCH);
            newRightRearTarget = rightRear.getCurrentPosition() + (int)(inches * COUNTS_PER_INCH);
          
            // Goal of the previous math.
            leftFront.setTargetPosition(newLeftFrontTarget);
            rightFront.setTargetPosition(newRightFrontTarget);
            leftRear.setTargetPosition(newLeftRearTarget);
            rightRear.setTargetPosition(newRightRearTarget);

            // Turn On RUN_TO_POSITION. Motor will run until it hits position.
            leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion. Turns the motors on.
            period.reset();
            leftFront.setPower(Math.abs(speed));
            rightFront.setPower(Math.abs(speed));
            leftRear.setPower(Math.abs(speed));
            rightRear.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.

		// Dangerous while loop.
            while ((period.seconds() < timeoutS) &&
                    (leftFront.isBusy() && rightFront.isBusy() && leftRear.isBusy() && rightRear.isBusy() )) {
                   // Wait for Sequence to complete
            }

            // Stop all motion; It is optional. It is better to comment this out to make the robot more effiecent and not coast (sliding).
            leftFront.setPower(0);
            rightFront.setPower(0);
            leftRear.setPower(0);
            rightRear.setPower(0);

            // Turn off RUN_TO_POSITION. It is optional.It is better to comment this out to make the robot more effiecent and not coast (sliding).
            leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            leftRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            rightRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            //  sleep(250);   // optional pause after each move
        }
    }

    public void pointTurnDegrees(double speed,
                                 double deg,
                                 double timeoutS) {

        int newLeftFrontTarget;
        int newRightFrontTarget;
        int newLeftRearTarget;
        int newRightRearTarget;

        // Reverse inches. Biggest difference.
        deg = deg * 3.0;

        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Set to Limit of DRIVE_SPEED
        if (Math.abs(speed) > DRIVE_SPEED) {
            speed = DRIVE_SPEED; //
        }

        // Ensure that the opmode is still active
        //if (opModeIsActive()) {
        if (true) {       // Swapped out to include in MaristBaseRobot

            // Determine new target position, and pass to motor controller. Notice the difference.
            newLeftFrontTarget = leftFront.getCurrentPosition() + (int)(deg * COUNTS_PER_DEGREE);
            newRightFrontTarget = rightFront.getCurrentPosition() - (int)(deg * COUNTS_PER_DEGREE);
            newLeftRearTarget = leftRear.getCurrentPosition() + (int)(deg * COUNTS_PER_DEGREE);
            newRightRearTarget = rightRear.getCurrentPosition() - (int)(deg * COUNTS_PER_DEGREE);
          
            //
            leftFront.setTargetPosition(newLeftFrontTarget);
            rightFront.setTargetPosition(newRightFrontTarget);
            leftRear.setTargetPosition(newLeftRearTarget);
            rightRear.setTargetPosition(newRightRearTarget);

            // Turn On RUN_TO_POSITION
            leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            period.reset();
            leftFront.setPower(Math.abs(speed));
            rightFront.setPower(Math.abs(speed));
            leftRear.setPower(Math.abs(speed));
            rightRear.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while ((period.seconds() < timeoutS) &&
                    (leftFront.isBusy() && rightFront.isBusy() && leftRear.isBusy() && rightRear.isBusy() )) {
                   // Wait for Sequence to complete
            }

            // Stop all motion;
            leftFront.setPower(0);
            rightFront.setPower(0);
            leftRear.setPower(0);
            rightRear.setPower(0);

            // Turn off RUN_TO_POSITION
            leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            leftRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            rightRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            //  sleep(250);   // optional pause after each move
        }

    }

    public void strafeInches(double speed,
                             double inches,
                             double timeoutS) {
        int newLeftFrontTarget;
        int newRightFrontTarget;
        int newLeftRearTarget;
        int newRightRearTarget;

        // Reverse inches
        inches = inches * -1 * (12.0/29);

        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Set to Limit of DRIVE_SPEED
        if (Math.abs(speed) > DRIVE_SPEED) {
            speed = DRIVE_SPEED; //
        }

        // Ensure that the opmode is still active
        //if (opModeIsActive()) {
        if (true) {       // Swapped out to include in MaristBaseRobot

            // Determine new target position, and pass to motor controller
            newLeftFrontTarget = leftFront.getCurrentPosition() - (int)(inches * COUNTS_PER_INCH);
            newRightFrontTarget = rightFront.getCurrentPosition() + (int)(inches * COUNTS_PER_INCH);
            newLeftRearTarget = leftRear.getCurrentPosition() + (int)(inches * COUNTS_PER_INCH);
            newRightRearTarget = rightRear.getCurrentPosition() - (int)(inches * COUNTS_PER_INCH);
          
            //
            leftFront.setTargetPosition(newLeftFrontTarget);
            rightFront.setTargetPosition(newRightFrontTarget);
            leftRear.setTargetPosition(newLeftRearTarget);
            rightRear.setTargetPosition(newRightRearTarget);

            // Turn On RUN_TO_POSITION
            leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            period.reset();
            leftFront.setPower(Math.abs(speed));
            rightFront.setPower(Math.abs(speed));
        leftRear.setPower(Math.abs(speed));
        rightRear.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while ((period.seconds() < timeoutS) &&
                    (leftFront.isBusy() && rightFront.isBusy() && leftRear.isBusy() && rightRear.isBusy() )) {
                   // Wait for Sequence to complete
            }

            // Stop all motion;
            leftFront.setPower(0);
            rightFront.setPower(0);
            leftRear.setPower(0);
            rightRear.setPower(0);

            // Turn off RUN_TO_POSITION
            leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            leftRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            rightRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            //  sleep(250);   // optional pause after each move
        }
    }

    public void leftArmMotorDeg(double speed,
                            double deg,
                            double timeoutS) {
        int target;

        leftArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Set to Limit of DRIVE_SPEED
        if (Math.abs(speed) > DRIVE_SPEED) {
            speed = DRIVE_SPEED; //
        }

        // Ensure that the opmode is still active
        //if (opModeIsActive()) {
        if (true) {       // Swapped out to include in MaristBaseRobot

            // Determine new target position, and pass to motor controller
            target = leftArm.getCurrentPosition() + (int)(deg * COUNTS_PER_DEGREE);
            leftArm.setTargetPosition(target);

            // Turn On RUN_TO_POSITION
            leftArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            period.reset();
            leftArm.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while ((period.seconds() < timeoutS) &&
                    leftArm.isBusy()) {
                // Wait for Sequence to complete
            }

            // Stop all motion: Comment out if you want Motor to hold position
            //leftArm.setPower(0);

            // Turn off RUN_TO_POSITION: Comment out if you want Motor to hold position
            //leftArm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            //  sleep(250);   // optional pause after each move
        }

    }

    public void rightArmMotorDeg(double speed,
                             double deg,
                             double timeoutS) {
        int target;

        rightArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        rightArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Set to Limit of DRIVE_SPEED
        if (Math.abs(speed) > DRIVE_SPEED) {
            speed = DRIVE_SPEED; //
        }

        // Ensure that the opmode is still active
        //if (opModeIsActive()) {
        if (true) {       // Swapped out to include in MaristBaseRobot

            // Determine new target position, and pass to motor controller
            target = rightArm.getCurrentPosition() + (int)(deg * COUNTS_PER_DEGREE);
            rightArm.setTargetPosition(target);

            // Turn On RUN_TO_POSITION
            rightArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            period.reset();
            rightArm.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while ((period.seconds() < timeoutS) &&
                    rightArm.isBusy()) {
                // Wait for Sequence to complete
            }

            // Stop all motion: Comment out if you want Motor to hold position
            //rightArm.setPower(0);

            // Turn off RUN_TO_POSITION: Comment out if you want Motor to hold position
            //rightArm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            //  sleep(250);   // optional pause after each move
        }

    }
    
    // For 2020: Add Commands from API for Engineering Java Robot Simulation
    // These are deprecated for 2022
    public void moveDistance(double inches, double speed) {
        driveStraightInches(speed, inches, 10);
    }
    
    public void turnAngle(double angle, double speed) {
        pointTurnDegrees(speed, angle, 10);
    }

    public void strafeInches(double inches, double speed) {
    strafeInches(speed, inches, 10);
    }

    // Additional Movement Methods - July 2023
    // Matches Java Karel Format
    // Overloaded Methods with (inches, speed) and (inches) options

    public void move(double inches, double speed) {
        driveStraightInches(speed, inches, 10);
    }

    public void move(double inches) {
        driveStraightInches(DRIVE_SPEED, inches, 10);
    }

    public void strafe(double inches, double speed) {
        strafeInches(speed, inches, 10);
    }

    public void strafe(double inches) {
        strafeInches(DRIVE_SPEED, inches, 10);
    }

    public void turnLeft(double degrees, double speed) {
        pointTurnDegrees(speed, degrees, 10);
    }

    public void turnLeft(double degrees) {
        pointTurnDegrees(TURN_SPEED, degrees, 10);
    }

    public void turnRight(double degrees, double speed) {
        pointTurnDegrees(speed, -degrees, 10);
    }

    public void turnRight(double degrees) {
        pointTurnDegrees(TURN_SPEED, -degrees, 10);
    }

    public void turn(double degrees, double speed) {
        pointTurnDegrees(speed, degrees, 10);
    }

    public void turn(double degrees) {
        pointTurnDegrees(TURN_SPEED, degrees, 10);
    }



    // Functions for Color Sensor - July 2021
    
    public float [] getColorValues() {
        NormalizedRGBA colors = colorSensor.getNormalizedColors();
        float [] output = new float [3];
        output[0] = colors.red;
        output[1] = colors.blue;
        output[2] = colors.green;
        return output;
    }
    
    public double getRed() {
        NormalizedRGBA colors = colorSensor.getNormalizedColors();
        return colors.red;
    }
    
    public double getGreen() {
        NormalizedRGBA colors = colorSensor.getNormalizedColors();
        return colors.green;
    }
    
    public double getBlue() {
        NormalizedRGBA colors = colorSensor.getNormalizedColors();
        return colors.blue;
    }
    
    public float getIntensity() {
        NormalizedRGBA colors = colorSensor.getNormalizedColors();
        float[] hsvValues = new float[3];
        Color.colorToHSV(colors.toColor(), hsvValues);
        return hsvValues[2];
    }

    // Added by Mr. Michaud 19 Sep 22
    public float getHue() {
        NormalizedRGBA colors = colorSensor.getNormalizedColors();
        float[] hsvValues = new float[3];
        Color.colorToHSV(colors.toColor(), hsvValues);
        return hsvValues[0];
    }
    //added by Cole Saunders 22 Oct 23
    public double getOrientation(){
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        return orientation.getYaw(AngleUnit.DEGREES)-offset;
    }
    public double resetImu(){
        offset=  imu.getRobotYawPitchRollAngles().getYaw((AngleUnit.DEGREES));
        return offset;
    }

    public void driveFieldCentric(double gamepadXPow, double gamepadYPow, double gamepadRotPow, double DRIVE_SPEED){ //Does not work.

        double gamepadTheta = Math.atan2(gamepadYPow, gamepadXPow);
        double diffTheta = gamepadTheta - Math.toRadians(getOrientation());

        double xpow= Math.cos(diffTheta ) *DRIVE_SPEED;
        double ypow= Math.sin(diffTheta)*DRIVE_SPEED;
        double rotpow = gamepadRotPow*DRIVE_SPEED;

        double leftRearPower = ypow+ xpow- rotpow ;
        double leftFrontPower = ypow- xpow- rotpow ;
        double rightRearPower = ypow- xpow+ rotpow ;
        double rightFrontPower = ypow+ xpow+ rotpow ;
            
        leftFront.setPower(leftFrontPower);
        leftRear.setPower(leftRearPower);
        rightFront.setPower(rightFrontPower);
        rightRear.setPower(rightRearPower);

        
    }

    public void driveStrafer(double leftX, double leftY, double rightX) {
			    (leftY is for Forward, leftX is for Strafing, and rightX is for Turning)
        double leftFrontPower = leftY - leftX - rightX;
        double leftRearPower = leftY + leftX - rightX;
        double rightFrontPower = leftY + leftX + rightX;
        double rightRearPower = leftY - leftX + rightX;

        leftFront.setPower(leftFrontPower);
        leftRear.setPower(leftRearPower);
        rightFront.setPower(rightFrontPower);
        rightRear.setPower(rightRearPower);

    }

    public void driveTank(double leftY, double rightY) {

        leftFront.setPower(leftY);
        leftRear.setPower(leftY);
        rightFront.setPower(rightY);
        rightRear.setPower(rightY);

    }
    
// ~ !IMPORTANT! This is the partner bracket to the MaristBaseRobot2024_Quad class. 
// ~ DO NOT DELETE!

}


