package org.firstinspires.ftc.teamcode;

import android.app.Activity;
import android.hardware.Sensor;
import android.view.View;

import com.qualcomm.hardware.adafruit.AdafruitBNO055IMU;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsDigitalTouchSensor;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cIrSeekerSensorV3;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;
import com.qualcomm.robotcore.hardware.DigitalChannelController;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.I2cDevice;
import com.qualcomm.robotcore.hardware.IrSeekerSensor;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.hardware.UltrasonicSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * This is NOT an opmode.
 *
 * This class can be used to define all the specific hardware for a single robot.
 * In this case that robot is a NeilPushbot.
 * See PushbotTeleopTank_Iterative and others classes starting with "NeilPushbot" for usage examples.
 *
 * This hardware class assumes the following device names have been configured on the robot:
 * Note:  All names are lower case and some have single spaces between words.
 *
 * Motor channel:  Left  drive motor:        "left_drive"
 * Motor channel:  Right drive motor:        "right_drive"
 * Motor channel:  Manipulator drive motor:  "left_arm"
 * Servo channel:  Servo to open left claw:  "left_hand"
 * Servo channel:  Servo to open right claw: "right_hand"
 */
public class NeilPushbot
{
    /* Public OpMode members. */
    public DcMotor  forklift            = null;
    public DcMotor  right_balllauncher  = null;
    public Servo    button_pusher       = null;
    public Servo      ball_feeder       = null;
    public OpticalDistanceSensor frontUSensor    = null;
    public IrSeekerSensor    backUSensor     = null;
    public ColorSensor right_color_sensor   = null;
    public ColorSensor  left_color_sensor   = null;
    public DcMotor    back_left_motor        = null;
    public DcMotor    back_right_motor       = null;
    public DcMotor    front_left_motor       = null;
    public DcMotor    front_right_motor      = null;
    public DcMotor      right_motor         = null;
    public DcMotor      left_motor          = null;
    public OpticalDistanceSensor lineSensor = null;
    public ModernRoboticsI2cGyro gyro   = null;
    public CRServo forkliftGrabber      = null;
    public OpticalDistanceSensor ODS    = null;
    public ModernRoboticsDigitalTouchSensor wallDetector = null;
    public static final double MID_SERVO       =  0.5 ;
    public static final double ARM_UP_POWER    =  0.45 ;
    public static final double ARM_DOWN_POWER  = -0.45 ;

    /* local OpMode members. */
    HardwareMap hwMap           =  null;
    private ElapsedTime period  = new ElapsedTime();

    /* Constructor */
    public NeilPushbot(){


    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {

        // Save reference to Hardware map
        hwMap = ahwMap;

        // Define and Initialize Motors
        forklift            = hwMap.dcMotor.get("forklift");
        right_balllauncher  = hwMap.dcMotor.get("ball_launcher");
        button_pusher       = hwMap.servo.get("button_pusher");
        ball_feeder         = hwMap.servo.get("ball_feeder");
        forkliftGrabber     = hwMap.crservo.get("forklift_grabber");
//        left_motor          = hwMap.dcMotor.get("left_motor");
//        right_motor         = hwMap.dcMotor.get("right_motor");
        front_right_motor   = hwMap.dcMotor.get("front_right_motor");
        front_left_motor    = hwMap.dcMotor.get("front_left_motor");
        back_right_motor    = hwMap.dcMotor.get("back_right_motor");
        back_left_motor     = hwMap.dcMotor.get("back_left_motor");
        gyro = (ModernRoboticsI2cGyro)hwMap.gyroSensor.get("gyro");
        lineSensor        = hwMap.opticalDistanceSensor.get("frontUSensor");
        wallDetector        = (ModernRoboticsDigitalTouchSensor)hwMap.touchSensor.get("Wall_Detector");
        //leftMotor.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors
        //rightMotor.setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD if using AndyMark motors
        front_right_motor.setDirection(DcMotorSimple.Direction.FORWARD);
        back_right_motor.setDirection(DcMotorSimple.Direction.FORWARD);
        front_left_motor.setDirection(DcMotorSimple.Direction.FORWARD);
        back_left_motor.setDirection(DcMotorSimple.Direction.FORWARD);

        // Set all motors to zero power
        forklift.setPower(0);
        right_balllauncher.setPower(0);


        // Set all motors to run without encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.
        forklift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        right_balllauncher.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Define and initialize ALL installed servos.
        button_pusher.setPosition(MID_SERVO);
        ball_feeder.setPosition(MID_SERVO);
//        left_motor.setPower(0);
//        right_motor.setPower(0);
        front_right_motor.setPower(0);
        front_left_motor.setPower(0);
        back_right_motor.setPower(0);
        back_left_motor.setPower(0);

        // values is a reference to the hsvValues array.

        // get a reference to the RelativeLayout so we can change the background
        // color of the Robot Controller app to match the hue detected by the RGB sensor.


        right_color_sensor  = hwMap.colorSensor.get("colorSensor");
        left_color_sensor   = hwMap.colorSensor.get("color_sensor2");

    }

    /***
     *
     * waitForTick implements a periodic delay. However, this acts like a metronome with a regular
     * periodic tick.  This is used to compensate for varying processing times for each cycle.
     * The function looks at the elapsed cycle time, and sleeps for the remaining time interval.
     *
     * @param periodMs  Length of wait cycle in mSec.
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
}

