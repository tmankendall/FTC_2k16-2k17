//@Disabled

package org.firstinspires.ftc.teamcode;
//CLEAN IT UP

import com.qualcomm.hardware.modernrobotics.ModernRoboticsDigitalTouchSensor;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;
import com.qualcomm.robotcore.hardware.DigitalChannelController;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

//<<<<<<< Updated upstream
//=======
//>>>>>>> Stashed changes
//import com.qualcomm.robotcore.hardware.CRServo;
//import com.qualcomm.robotcore.hardware.IrSeekerSensor;
//<<<<<<< Updated upstream
//=======

//>>>>>>> Stashed changes

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
public class RobotArmPushBot
{
    /* Public OpMode members. */
    public DcMotor extension    = null;
    public CRServo baseRotation = null;
    public CRServo sweeper      = null;
    private ElapsedTime period  = new ElapsedTime();

    public RobotArmPushBot(){


    }
    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap hwMap) {
        extension       = hwMap.dcMotor.get("extender");
        baseRotation    = hwMap.crservo.get("baseRotate");
        sweeper         = hwMap.crservo.get("sweeper");
        extension.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    }

    /***
     *
     * waitForTick implements a periodic delay. However, this acts like a metronome with a regular
     * periodic tick.  This is used to compensate for varying processing times for each cycle.
     * The function looks at the elapsed cycle time, and sleeps for the remaining time interval.
     *
     * @param periodMs  Length of wait cycle in mSec
     *                  */
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
