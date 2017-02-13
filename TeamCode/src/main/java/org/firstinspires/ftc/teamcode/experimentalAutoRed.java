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

@Autonomous(name="experimentalAutoRed", group="Andrew")  // @Autonomous(...) is the other common choice
//@Disabled
public class experimentalAutoRed extends LinearOpMode {

    /* Declare OpMode members. */
    private ElapsedTime runtime = new ElapsedTime();
    NeilPushbot robot = new NeilPushbot();
    double standardLightValue;
    double whiteLightValue = .2;
    //<<<<<<< Updated upstream
    int allRed = 1000;
    double power;
    //double angle = 35;
    //=======
    double blackLightValue = 0.023;
    double correction;
    double leftSpeed;
    double rightSpeed;
    double zAccumulated;

    //>>>>>>> Stashed changes
    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        robot.init(hardwareMap);

        telemetry.addData(">", "Gyro Calibrating. Do Not move!");
        telemetry.update();
        robot.gyro.calibrate();
        while (robot.gyro.isCalibrating()) {

        }
        robot.gyro.resetZAxisIntegrator();


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
        GyroTurn(-35);
        driveGyroStraight(-35, .3);
        GyroTurn(-55);
        followLine();
        pressRed();
        //fire();
        //sleep(7000);
        //verifyRed();
        //pressRed();
        //verifyRed();
        pressRedNoFire();
    }

    private void verifyRed() {
        double redColorRight = robot.right_color_sensor.red();
        double redColorLeft = robot.left_color_sensor.red();
        double blueColorRight = robot.right_color_sensor.blue();
        double blueColorLeft = robot.left_color_sensor.blue();
        if (robot.right_color_sensor.red()>robot.right_color_sensor.blue() && robot.left_color_sensor.red() > robot.left_color_sensor.blue()){
            Drive2ndBeacon();
        }
        else{
            pressRedNoFire();
        }
    }

    private void ColorConfirm() {

        double redColorRight = robot.right_color_sensor.red();
        double redColorLeft = robot.left_color_sensor.red();
        double blueColorRight = robot.right_color_sensor.blue();
        double blueColorLeft = robot.left_color_sensor.blue();
        while (redColorRight < blueColorRight && redColorLeft < blueColorLeft) {
            pressRed();
        }
        Drive2ndBeacon();
    }

    private void pressRed() {
        halt();
        reverse(.1);
        idle();
        sleep(200);
        halt();
        double redColorLeft = robot.left_color_sensor.red();
        double blueColorLeft = robot.left_color_sensor.blue();
        if (redColorLeft > blueColorLeft) {
            halt();
            fire();
            Drive2ndBeacon();
        } else if (blueColorLeft > redColorLeft) {
            forward(.1);
            sleep(1000);
            reverse(.1);
            sleep(1000);
            halt();
            fire();
            sleep(2000); //change accordingly
            forward(.1);
            sleep(1500);
            verifyRed();

        }
        halt();

    }
    private void pressRedNoFire() {
        halt();
        reverse(.1);
        idle();
        sleep(200);
        halt();
        double redColorLeft = robot.left_color_sensor.red();
        double blueColorLeft = robot.left_color_sensor.blue();
        if (redColorLeft > blueColorLeft) {
            halt();
            fire();
            Drive2ndBeacon();

        } else if (blueColorLeft > redColorLeft) {
            forward(.1);
            sleep(1000);
            reverse(.1);
            sleep(1000);
            halt();
            //fire();
            sleep(6000); //change accordingly
            forward(.1);
            sleep(1500);
            verifyRed();

        }
        halt();

    }
    /*private void nextBeacon() {
        reverse(.1);
        sleep(300);
        Drive2ndBeacon();
    }*/

    private void Drive2ndBeacon() {
        robot.back_left_motor.setPower(-1);
        robot.front_right_motor.setPower(-1);
        robot.back_right_motor.setPower(1);
        robot.front_left_motor.setPower(1);
        reverse(.1);
        sleep(300);
        if (robot.lineSensor.getLightDetected() == whiteLightValue) {
            halt();
            followLine();
        }
        else{
            robot.front_left_motor.setPower(1);
            robot.back_left_motor.setPower(-1);
            robot.front_right_motor.setPower(1);
            robot.back_right_motor.setPower(-1);
        }
        //This technically works but if we have issues I can do a more advanced version which is better.
        double initialHeading = robot.gyro.getHeading();
        while (robot.lineSensor.getLightDetected() > whiteLightValue + .1 || robot.lineSensor.getLightDetected() < whiteLightValue + .1) {
            if (initialHeading - robot.gyro.getHeading() > 5) {
                robot.back_left_motor.setPower(robot.back_left_motor.getPower() + .01);
                robot.back_right_motor.setPower(robot.back_right_motor.getPower() - .01);
            } else if (robot.gyro.getHeading() - initialHeading > 5) {
                robot.back_left_motor.setPower(robot.back_left_motor.getPower() - .01);
                robot.back_right_motor.setPower(robot.back_right_motor.getPower() + .01);
            }
        }
        halt();
        followLine();
    }

    private void goRight(int time) {
        robot.back_left_motor.setPower(-1);
        robot.front_right_motor.setPower(-1);
        robot.back_right_motor.setPower(1);
        robot.front_left_motor.setPower(1);
        sleep(time);
        robot.back_left_motor.setPower(0);
        robot.front_right_motor.setPower(0);
        robot.back_right_motor.setPower(0);
        robot.front_left_motor.setPower(0);
    }

    private void goLeft(int time) {
        robot.back_left_motor.setPower(1);
        robot.front_right_motor.setPower(1);
        robot.back_right_motor.setPower(-1);
        robot.front_left_motor.setPower(-1);
        sleep(time);
        robot.back_left_motor.setPower(0);
        robot.front_right_motor.setPower(0);
        robot.back_right_motor.setPower(0);
        robot.front_left_motor.setPower(0);
    }

    private void reverse(double power) {
        robot.front_left_motor.setPower(-power);
        robot.front_right_motor.setPower(-power);
        robot.back_left_motor.setPower(-power);
        robot.back_right_motor.setPower(-power);
        idle();
        return;
    }

    private void forward(double power) {
        robot.front_left_motor.setPower(power);
        robot.front_right_motor.setPower(power);
        robot.back_left_motor.setPower(power);
        robot.back_right_motor.setPower(power);
        idle();
        return;
    }

    private void halt() {
        robot.back_left_motor.setPower(0);
        robot.back_right_motor.setPower(0);
        robot.front_left_motor.setPower(0);
        robot.front_right_motor.setPower(0);
        idle();
        return;
    }

    private void driveGyroStraight(double angle, double powerGyro) {
        double target = angle;  //Starting direction
        zAccumulated = robot.gyro.getIntegratedZValue();  //Current direction
        //tk trying to maybe fix things
        //drive(1,1);
        while (robot.lineSensor.getLightDetected() < .2 - .02) {
            leftSpeed = powerGyro + (zAccumulated - target) / 100;  //Calculate speed for each side
            rightSpeed = powerGyro - (zAccumulated - target) / 100;  //See Gyro Straight video for detailed explanation
            leftSpeed = Range.clip(leftSpeed, -1, 1);
            rightSpeed = Range.clip(rightSpeed, -1, 1);
            drive(leftSpeed, rightSpeed);
            idle();
        }
        //drive(0,0);
    }

    private void driveForTime(double time) {
        double desiredTime = time;
        double initialTime = runtime.milliseconds();
        double currentTime = runtime.milliseconds();
        int desiredAngle = robot.gyro.getHeading();
        int currentAngle;
        double rightPower = 1;
        double leftPower = 1;
        robot.front_right_motor.setPower(rightPower);
        robot.front_left_motor.setPower(leftPower);
        robot.back_right_motor.setPower(rightPower);
        robot.back_left_motor.setPower(leftPower);
        while (desiredTime > (currentTime - initialTime)) {
            currentAngle = robot.gyro.getHeading();
            if (currentAngle > desiredAngle) {
                leftPower -= .01;
                rightPower = .5;
            } else if (currentAngle < desiredAngle) {
                rightPower -= .01;
                leftPower = .5;
            }
            sleep(1);
        }
        robot.front_left_motor.setPower(0);
        robot.front_right_motor.setPower(0);
        robot.back_left_motor.setPower(0);
        robot.back_right_motor.setPower(0);
    }

    private void followLine() {
        double counter = 0;
        double currentLightDetected = robot.lineSensor.getLightDetected();
        correction = (whiteLightValue - robot.lineSensor.getLightDetected());
        robot.back_right_motor.setPower(0);
        robot.back_left_motor.setPower(0);
        robot.front_left_motor.setPower(0);
        robot.front_right_motor.setPower(0);
        idle();
        while (robot.wallDetector.isPressed() == false) {
            correction = (whiteLightValue - robot.lineSensor.getLightDetected());
            if (robot.lineSensor.getLightDetected() > blackLightValue + .1) {
                telemetry.addData("I found the Line", "");
                telemetry.update();
                if (correction <= 0) {
                    leftSpeed = .075d - correction;
                    rightSpeed = .075d;
                    drive(leftSpeed, rightSpeed);
                } else {
                    leftSpeed = .075d;
                    rightSpeed = .075d + correction;
                    drive(leftSpeed, rightSpeed);
                }

            }
            idle();
        }
        if (robot.wallDetector.isPressed() == true) {

            halt();
        }


    }


    private void drive(double left, double right) {
        robot.front_left_motor.setPower(left);
        robot.front_right_motor.setPower(right);
        robot.back_left_motor.setPower(left);
        robot.back_right_motor.setPower(right);
    }

    public void GyroTurn(int target) {
        zAccumulated = robot.gyro.getIntegratedZValue();  //Set variables to gyro readings
        double turnSpeed = 0.15;

        while (Math.abs(zAccumulated - target) > 3 && opModeIsActive()) {  //Continue while the robot direction is further than three degrees from the target
            if (zAccumulated > target) {  //if gyro is positive, we will turn right
                //leftSpeed = turnSpeed;
                //rightSpeed = -turnSpeed;
                drive(turnSpeed, -turnSpeed);
            }

            if (zAccumulated < target) {  //if gyro is positive, we will turn left
                //leftSpeed = -turnSpeed;
                // rightSpeed = turnSpeed;
                drive(-turnSpeed, turnSpeed);
            }

            zAccumulated = robot.gyro.getIntegratedZValue();  //Set variables to gyro readings
//            telemetry.addData("accu", String.format("%03d", zAccumulated));
            telemetry.update();
        }


        drive(0, 0);

    }

    private void fire() {
        ElapsedTime runtime = new ElapsedTime();
        runtime.reset();
        double n = 60;
        for (int i = 0; i < 1; i += .1) {
            robot.right_balllauncher.setPower(i);
        }
        sleep(100);
        robot.ball_feeder.setPosition(120.0 / 180.0 + n / 180.0);
        sleep(500);
        robot.ball_feeder.setPosition(.5);
        sleep(1000);
        robot.ball_feeder.setPosition(120.0 / 180.0 + n / 180.0);
        sleep(500);
        robot.ball_feeder.setPosition(.5);
        for (int j = 1; j > 0; j -= .1) {
            robot.right_balllauncher.setPower(j);
        }
        return;


    }



    }

