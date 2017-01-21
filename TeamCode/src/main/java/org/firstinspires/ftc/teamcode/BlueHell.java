/*
Copyright (c) 2016 Robert Atkinson

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of Robert Atkinson nor the names of his contributors may be used to
endorse or promote products derived from this software without specific prior
written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESSFOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a PushBot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@Autonomous(name="AutonomousBLUEHELL", group="Andrew")  // @Autonomous(...) is the other common choice
//@Disabled
public class BlueHell extends LinearOpMode {

    /* Declare OpMode members. */
    private ElapsedTime runtime = new ElapsedTime();
    NeilPushbot robot = new NeilPushbot();
    boolean right = true;
    boolean angled = true;
    boolean blue = true;
    double pushingRight = 1;
    double pushingLeft = .3;

    // DcMotor rightMotor = null;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        robot.init(hardwareMap);

        telemetry.addData(">", "Gyro Calibrating. Do Not move!");
        telemetry.update();
        robot.gyro.calibrate();

        // make sure the gyro is calibrated.
        while (!isStopRequested() && robot.gyro.isCalibrating()) {
            sleep(50);
            idle();
        }

        telemetry.addData(">", "Gyro Calibrated.  Press Start.");
        telemetry.update();

        /* eg: Initialize the hardware variables. Note that the strings used here as parameters
         * to 'get' must correspond to the names assigned during the robot configuration
         * step (using the FTC Robot Controller app on the phone).
         */
        // leftMotor  = hardwareMap.dcMotor.get("left_drive");
        // rightMotor = hardwareMap.dcMotor.get("right_drive");

        // eg: Set the drive motor directions:
        // "Reverse" the motor that runs backwards when connected directly to the battery
        // leftMotor.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors
        // rightMotor.setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD if using AndyMark motors

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();
        robot.right_motor.setPower(1);
        robot.left_motor.setPower(1);
        sleep(10000);
        robot.right_motor.setPower(0);
        robot.left_motor.setPower(0);
//        driveUntilF(7);
//        MakingThingBlue();
    }
}
//
//
//    // This code checks what side
//    private void MakingThingBlue() {
//        int surelyBlue = 400;
//        if (robot.right_color_sensor.red() > robot.left_color_sensor.red()) {
//            robot.button_pusher.setPosition(pushingLeft);
//            while (robot.right_color_sensor.red() > surelyBlue) {
//                robot.right_motor.setPower(.5);
//                robot.left_motor.setPower(.5);
//                sleep(10);
//            }
//            robot.right_motor.setPower(-.2);
//            robot.left_motor.setPower(-.2);
//            sleep(300);
//            robot.left_motor.setPower(0);
//            robot.right_motor.setPower(0);
//        } else {
//            robot.button_pusher.setPosition(pushingRight);
//            while (robot.left_color_sensor.red() > surelyBlue) {
//                robot.right_motor.setPower(.5);
//                robot.left_motor.setPower(.5);
//                sleep(10);
//            }
//            robot.right_motor.setPower(-.2);
//            robot.left_motor.setPower(-.2);
//            sleep(300);
//            robot.left_motor.setPower(0);
//            robot.right_motor.setPower(0);
//        }
//    }
//
//
//    private void driveUntilF(int distanceNeeded) {
//        double odsReadingRaw;
//        double odsReadingLinear;
//        double distanceFromWall = distanceNeeded;
//        float sensorDifference = 0;
//        float temporaryDifference = 0;
//        int maxSpeed = 15;
//        double average = 0;
//        int counter = 0;
//        double totalReads = 0;
//        double[] pastReadings = new double[5];
//        odsReadingRaw = robot.frontUSensor.getRawLightDetected();
//        odsReadingLinear = Math.pow(odsReadingRaw, -0.5);
//        while ((average > distanceFromWall + 1) && sensorDifference < 100) {
//            if (counter < 5) {
//                counter++;
//            }
//            if (counter > 1) {
//                for (int i = 1; i < counter; i++) {
//                    pastReadings[i] = pastReadings[i - 1];
//                }
//            }
//            pastReadings[0] = odsReadingLinear;
//            for (int i = 0; i < counter; i++) {
//                totalReads += pastReadings[i];
//            }
//            average = (totalReads / counter);
//            if (average > distanceFromWall + 1) {
//                robot.right_motor.setPower(maxSpeed);
//                robot.left_motor.setPower(maxSpeed);
//            } else {
//                robot.right_motor.setPower(average - distanceFromWall);
//                robot.left_motor.setPower(average - distanceFromWall);
//            }
//            temporaryDifference = robot.right_color_sensor.red() - robot.left_color_sensor.red();
//            sensorDifference = Math.abs(temporaryDifference);
//            odsReadingRaw = robot.frontUSensor.getRawLightDetected();
//            odsReadingLinear = Math.pow(odsReadingRaw, -0.5);
//        }
//
//    }