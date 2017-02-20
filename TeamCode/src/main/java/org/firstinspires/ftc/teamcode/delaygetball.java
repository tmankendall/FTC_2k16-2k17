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

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;


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

@Autonomous(name="delaygetball", group="Andrew")  // @Autonomous(...) is the other common choice
//@Disabled
public class delaygetball extends LinearOpMode {

    /* Declare OpMode members. */
    private ElapsedTime runtime = new ElapsedTime();
    NeilPushbot robot = new NeilPushbot();
    double standardLightValue = 0;
    double followingValue = .015;
    double whiteLightValue = .015;
    float redColorRight;
    float blueColorRight;
    float redColorLeft;
    float blueColorLeft;
    float blueColor;
    float redColor;

    double blackLightValue = 0.0;
    double correction;
    double leftSpeed;
    double rightSpeed;
    double zAccumulated;
    double currentValue = 0;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        robot.init(hardwareMap);

        robot.front_left_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.front_right_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.back_left_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.back_right_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        telemetry.addData(">", "Gyro Calibrating. Do Not move!");
        telemetry.update();
        robot.gyro.calibrate();
        while (robot.gyro.isCalibrating()) {

        }
        robot.gyro.resetZAxisIntegrator();
//.012

        // make sure the gyro is calibrated.
//        while (!isStopRequested() && robot.gyro.isCalibrating())  {
//            sleep(50);
//            idle();
//        }
        telemetry.addData(">", "Gyro Calibrated.  Press Start.");
        telemetry.update();
        robot.front_left_motor.setDirection(DcMotorSimple.Direction.FORWARD);
        robot.back_left_motor.setDirection(DcMotorSimple.Direction.FORWARD);
        robot.front_right_motor.setDirection(DcMotorSimple.Direction.REVERSE);
        robot.back_right_motor.setDirection(DcMotorSimple.Direction.REVERSE);
        waitForStart();
        runtime.reset();
        robot.ball_feeder.setPosition(47.0 / 180.0);
        robot.ForkliftGrabber.setPosition(1);
        fire();
        sleep(10000);
        driveForTime(0,3000);
    }
    private void driveGyroStraight(double angle, double powerGyro) {
        telemetry.addLine("we made it here");
        telemetry.update();
        double target = angle;  //Starting direction
        zAccumulated = robot.gyro.getIntegratedZValue();  //Current direction



        double[] values = new double[5];
        int n = 0;

        for(int i = 0; i < 5; i++)
        {
            values[i] = 0;
            idle();
        }
        double average = 0;


        while (opModeIsActive() && average < followingValue) {

            average = 0;
            values[n] = robot.lineSensor.getRawLightDetected();

            if(n == 5)
            {
                n = 0;
            }

            for(int i = 0; i < 5; i++)
            {
                average += values[i];
            }
            average = average/5.0;

            leftSpeed = powerGyro - (zAccumulated - target) / 100.0;  //Calculate speed for each side
            rightSpeed = powerGyro + (zAccumulated - target) / 100.0;  //See Gyro Straight video for detailed explanation
            leftSpeed = Range.clip(leftSpeed, -1, 1);
            rightSpeed = Range.clip(rightSpeed, -1, 1);
            drive(leftSpeed, rightSpeed);
            zAccumulated = robot.gyro.getIntegratedZValue();
            telemetry.addData("Driving, ods value is", robot.lineSensor.getRawLightDetected());
            telemetry.addData("Current Integrated Z value is:", zAccumulated);
            telemetry.update();
            idle();
        }
        //drive(0,0);
    }
    private void drive(double left, double right) {
        robot.front_left_motor.setPower(left);
        robot.front_right_motor.setPower(right);
        robot.back_left_motor.setPower(left);
        robot.back_right_motor.setPower(right);
    }
    private void fire()
    {

        robot.ball_feeder.setPosition(47.0/180.0);

        for (float i = 0; i < 1; i += .01) {
            robot.right_balllauncher.setPower(i);
            sleep(20);
            idle();
        }
        robot.right_balllauncher.setPower(1);
        sleep(800);

        robot.ball_feeder.setPosition(177.0/180.0);
        sleep(100);

        // robot.ball_feeder.setPosition(20/180);
        robot.ball_feeder.setPosition(47.0/180.0);
        sleep(100);
        robot.ball_feeder.setPosition(177.0/180.0);
        sleep(100);
        robot.ball_feeder.setPosition(47.0/180.0);
        for (float j = 1; (j > 0 && opModeIsActive()); j -= .01) {
            robot.right_balllauncher.setPower(j);
            sleep(20);
            idle();
        }
        return;


    }
    private void driveForTime(double angle, double time) {

        //double initialTime = runtime.milliseconds();
        double initialTime = runtime.milliseconds();
        double target = angle;  //Starting direction
        zAccumulated = robot.gyro.getIntegratedZValue();
        double currentTime = runtime.milliseconds();//Current direction
        double desiredTime = time + currentTime;
        //tk trying to maybe fix things
        //drive(1,1);
        while ((desiredTime-currentTime) > 0 && opModeIsActive()) {
            currentTime = runtime.milliseconds();
            leftSpeed = .3 - (zAccumulated - target) / 100.0;  //Calculate speed for each side
            rightSpeed = .3 + (zAccumulated - target) / 100.0;  //See Gyro Straight video for detailed explanation
            leftSpeed = Range.clip(leftSpeed, -1, 1);
            rightSpeed = Range.clip(rightSpeed, -1, 1);
            drive(leftSpeed, rightSpeed);
            zAccumulated = robot.gyro.getIntegratedZValue();
            telemetry.addData("Driving, ods value is", robot.lineSensor.getRawLightDetected());
            telemetry.update();
            idle();
        }

    }
}