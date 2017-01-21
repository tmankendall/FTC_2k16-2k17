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

@Autonomous(name="leftangledred", group="Andrew")  // @Autonomous(...) is the other common choice
//@Disabled
public class left_angled extends LinearOpMode {

    /* Declare OpMode members. */
    private ElapsedTime runtime = new ElapsedTime();
    NeilPushbot robot = new NeilPushbot();
    double pushingRight = 1;
    double pushingLeft =  .3;
    boolean right = false;
    boolean angled = true;
    boolean blue = false;

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
        while (!isStopRequested() && robot.gyro.isCalibrating())  {
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
        //driveUntilB(15);
        fire();
        fire();
        if (angled)
        {
            if (right)
            {
                turnDegrees(3);
                driveUntilF((8));
                turnDegrees(-3);
            }
            else
            {
                turnDegrees(-3);
                driveUntilF(8);
                turnDegrees(3);
            }
        }
        else
        {
            driveUntilF(8);
        }
//        turnRUntil(90);
//        driveUntilF(5);
        if (blue)
        {

            MakingThingBlue();
        }
        else
        {
            MakingThingRed();
        }


        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.update();

            // eg: Run wheels in tank mode (note: The joystick goes negative when pushed forwards)
            // leftMotor.setPower(-gamepad1.left_stick_y);
            // rightMotor.setPower(-gamepad1.right_stick_y);
            idle();
        }
    }
    private void turnRUntil(int headingChange)
    {
        robot.right_motor.setPower(0);
        robot.left_motor.setPower(.5);
        robot.gyro.resetZAxisIntegrator();
        // get the x, y, and z values (rate of change of angle).


        // get the heading info.
        // the Modern Robotics' gyro sensor keeps
        // track of the current heading for the Z axis only.
        double heading = robot.gyro.getHeading();
        double angleZ  = robot.gyro.getIntegratedZValue();
        while(robot.gyro.getHeading() - heading < headingChange)
        {
            sleep(10);
            idle();
        }
        robot.left_motor.setPower(0);
        robot.right_motor.setPower(0);

    }
    private void driveUntilF(int distanceNeeded)
    {
        double odsReadingRaw;
        double odsReadingLinear;
        double distanceFromWall = distanceNeeded;
        float sensorDifference = 0;
        float temporaryDifference = 0;
        int maxSpeed = 15;
        double average = 0;
        int counter = 0;
        double totalReads = 0;
        double[] pastReadings = new double[5];
        odsReadingRaw = robot.frontUSensor.getRawLightDetected();
        odsReadingLinear = Math.pow(odsReadingRaw, -0.5);
        while((average > distanceFromWall+ 1) && sensorDifference < 100) {
            if (counter < 5) {
                counter++;
            }
            if (counter > 1) {
                for (int i = 1; i < counter; i++) {
                    pastReadings[i] = pastReadings[i - 1];
                }
            }
            pastReadings[0] = odsReadingLinear;
            for (int i = 0; i < counter; i++) {
                totalReads += pastReadings[i];
            }
            average = (totalReads / counter);
            if (average > distanceFromWall + 1) {
                robot.right_motor.setPower(maxSpeed);
                robot.left_motor.setPower(maxSpeed);
            } else {
                robot.right_motor.setPower(average - distanceFromWall);
                robot.left_motor.setPower(average - distanceFromWall);
            }
            temporaryDifference = robot.right_color_sensor.red() - robot.left_color_sensor.red();
            sensorDifference = Math.abs(temporaryDifference);
            odsReadingRaw = robot.frontUSensor.getRawLightDetected();
            odsReadingLinear = Math.pow(odsReadingRaw, -0.5);
        }

    }
    private void turnDegrees(int degrees)
    {
        float initialDegree = robot.gyro.getHeading();
        if (degrees > 0) {
            while (robot.gyro.getHeading() - initialDegree < degrees) {
                robot.right_motor.setPower(.05);
            }
        }
        else
        {
            while (initialDegree - robot.gyro.getHeading() < Math.abs(degrees))
            {
                robot.left_motor.setPower(.05);
            }
            robot.right_motor.setPower(0);
        }
        robot.left_motor.setPower(0);
    }
    //    private void driveUntilB(int distanceNeeded)
//    {
//        robot.left_motor.setPower(1);
//        robot.right_motor.setPower(1);
//        while(takeRead(0) < distanceNeeded)
//        {
//            sleep(30);
//            idle();
//        }
//        robot.left_motor.setPower(0);
//        robot.right_motor.setPower(0);
//    }
    // This code checks what side
    private void MakingThingRed()
    {
        int surelyRed = 1400;
        if (robot.right_color_sensor.red() > robot.left_color_sensor.red())
        {
            robot.button_pusher.setPosition(pushingRight);
            while (robot.left_color_sensor.red() < surelyRed) {
                robot.right_motor.setPower(.5);
                robot.left_motor.setPower(.5);
                sleep(10);
            }
            robot.right_motor.setPower(-.2);
            robot.left_motor.setPower(-.2);
            sleep(300);
            robot.left_motor.setPower(0);
            robot.right_motor.setPower(0);
        }
        else
        {
            robot.button_pusher.setPosition(pushingLeft);
            while (robot.right_color_sensor.red() < surelyRed) {
                robot.right_motor.setPower(.5);
                robot.left_motor.setPower(.5);
                sleep(10);
            }
            robot.right_motor.setPower(-.2);
            robot.left_motor.setPower(-.2);
            sleep(300);
            robot.left_motor.setPower(0);
            robot.right_motor.setPower(0);
        }
    }
    // This code checks what side
    private void MakingThingBlue() {
        int surelyBlue = 400;
        if (robot.right_color_sensor.red() > robot.left_color_sensor.red()) {
            robot.button_pusher.setPosition(pushingLeft);
            while (robot.right_color_sensor.red() > surelyBlue) {
                robot.right_motor.setPower(.5);
                robot.left_motor.setPower(.5);
                sleep(10);
            }
            robot.right_motor.setPower(-.2);
            robot.left_motor.setPower(-.2);
            sleep(300);
            robot.left_motor.setPower(0);
            robot.right_motor.setPower(0);
        } else {
            robot.button_pusher.setPosition(pushingRight);
            while (robot.left_color_sensor.red() > surelyBlue) {
                robot.right_motor.setPower(.5);
                robot.left_motor.setPower(.5);
                sleep(10);
            }
            robot.right_motor.setPower(-.2);
            robot.left_motor.setPower(-.2);
            sleep(300);
            robot.left_motor.setPower(0);
            robot.right_motor.setPower(0);
        }
    }
    private void fire()
    {
        for(int i =0; i < 1; i+=.01)
        {

            robot.right_balllauncher.setPower(i);
            sleep(15);
        }
        robot.ball_feeder.setPosition(120);
        sleep(500);
        robot.ball_feeder.setPosition(90);
        robot.right_balllauncher.setPower(0);
    }
    private double takeRead(int ForB)
    {
        if(ForB == 1) {
            if (robot.backUSensor.signalDetected()) {
                // Display angle and strength
                telemetry.addData("Angle", robot.backUSensor.getAngle());
                telemetry.addData("Strength", robot.backUSensor.getStrength());
            } else {
                // Display loss of signal
                telemetry.addData("Seeker", "Signal Lost");
                return(robot.backUSensor.getSignalDetectedThreshold());
            }

            telemetry.update();
            return (robot.backUSensor.getStrength());
        }
        if (ForB == 0){
            telemetry.addData("Raw",    robot.frontUSensor.getRawLightDetected());
            telemetry.addData("Normal", robot.frontUSensor.getLightDetected());

            telemetry.update();
            return (robot.frontUSensor.getLightDetected());
        }
        else{
            throw (new IllegalArgumentException());
        }
    }
}
