package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.I2cDevice;
import com.qualcomm.robotcore.hardware.Servo;
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
    public DcMotor  sweeper             = null;
    public DcMotor  left_balllauncher   = null;
    public DcMotor  right_balllauncher  = null;
    public Servo    button_pusher       = null;
    public Servo      ball_launcher       = null;
    public CRServo    left_motor          = null;
    public CRServo    right_motor         = null;
    public I2cDevice ultrasonic2 = null;
    public GyroSensor gyro              = null;
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
        sweeper             = hwMap.dcMotor.get("sweeper");
        left_balllauncher   = hwMap.dcMotor.get("left_balllauncher");
        right_balllauncher  = hwMap.dcMotor.get("right_balllauncher");
        button_pusher       = hwMap.servo.get("button_pusher");
        ball_launcher       = hwMap.servo.get("ball_launcher");
        left_motor          = hwMap.crservo.get("left_motor");
        right_motor         = hwMap.crservo.get("right_motor");
        gyro                = hwMap.gyroSensor.get("gyro");
        //leftMotor.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors
        //rightMotor.setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD if using AndyMark motors

        // Set all motors to zero power
        forklift.setPower(0);
        sweeper.setPower(0);
        left_balllauncher.setPower(0);
        right_balllauncher.setPower(0);


        // Set all motors to run without encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.
        forklift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        sweeper.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        left_balllauncher.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        right_balllauncher.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Define and initialize ALL installed servos.
        button_pusher.setPosition(MID_SERVO);
        ball_launcher.setPosition(MID_SERVO);
        left_motor.setPower(0);
        right_motor.setPower(0);
        left_motor.setDirection(DcMotorSimple.Direction.FORWARD);
        right_motor.setDirection(DcMotorSimple.Direction.REVERSE);

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

