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

@Autonomous(name="ColorValues", group="Andrew")  // @Autonomous(...) is the other common choice
//@Disabled
public class ColorValues extends LinearOpMode {

    /* Declare OpMode members. */
    private ElapsedTime runtime = new ElapsedTime();
    NeilPushbot robot = new NeilPushbot();
    double standardLightValue = 0;
    double followingValue = .023;
    double whiteLightValue = .023;
    float redColorRight;
    float blueColorRight;
    float redColorLeft;
    float blueColorLeft;
    //<<<<<<< Updated upstream
    //int allRed = 1000;
    //double power;
    //double angle = 35;
    //=======
    double blackLightValue = 0.0;
    double correction;
    double leftSpeed;
    double rightSpeed;
    double zAccumulated;

    //>>>>>>> Stashed changes
    @Override
    public void runOpMode() {

        robot.init(hardwareMap);




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
        robot.ball_feeder.setPosition(47/180);
        while (opModeIsActive()) {
            getColors();
            idle();
        }

    }
    private void getColors(){
        getRightValueRed();
        getRightValueBlue();
        sleep(1000);
        getLeftValueBlue();
        getLeftValueRed();
        sleep(1000);
   }
    private void getRightValueRed(){
        robot.joshThisIsForYou.setDigitalChannelState(3, true);
        robot.joshThisIsForYou.setDigitalChannelState(4, false);
        redColorRight = robot.right_color_sensor.red();
        telemetry.addData("right red value is ", redColorRight);
        telemetry.update();

    }
    private void getLeftValueRed(){
        robot.joshThisIsForYou.setDigitalChannelState(4, true);
        robot.joshThisIsForYou.setDigitalChannelState(3, false);
        redColorLeft = robot.left_color_sensor.red();
        telemetry.addData("left red value is ", redColorLeft);
        telemetry.update();

    }
    private void getRightValueBlue(){
        robot.joshThisIsForYou.setDigitalChannelState(3, true);
        robot.joshThisIsForYou.setDigitalChannelState(4, false);
        blueColorRight = robot.right_color_sensor.blue();
        telemetry.addData("right blue value is ", blueColorRight);
        telemetry.update();
    }
    private void getLeftValueBlue(){
        robot.joshThisIsForYou.setDigitalChannelState(4, true);
        robot.joshThisIsForYou.setDigitalChannelState(3, false);
        blueColorLeft = robot.left_color_sensor.blue();
        telemetry.addData("left blue value is ", blueColorLeft);
        telemetry.update();
    }

    private void ColorAlignRed(){
        getColors();
        if (redColorRight > blueColorRight){
            while(blueColorLeft>redColorLeft){
                robot.back_left_motor.setPower(-1);
                robot.front_right_motor.setPower(-1);
                robot.back_right_motor.setPower(1);
                robot.front_left_motor.setPower(1);
            }

        }
        if (redColorLeft > blueColorLeft){
            while(blueColorRight> redColorRight){
                robot.back_left_motor.setPower(1);
                robot.front_right_motor.setPower(1);
                robot.back_right_motor.setPower(-1);
                robot.front_left_motor.setPower(-1);
            }
        }


    }
    private void ColorConfirm() {

        getColors();
        while (redColorRight < blueColorRight && redColorLeft < blueColorLeft) {
            pressRed();
        }
        Drive2ndBeacon();
    }
    private void verifyRed(){
        getColors();
        if (redColorLeft>blueColorLeft && redColorRight >blueColorRight){
            fire();
            halt();
        }
        else{
            pressRed();
        }

    }

    private void pressRed() {
        halt();
        reverse(.3);
        idle();
        sleep(200);
        halt();
        getColors();
        fire();
        sleep(5000);
        while (robot.wallDetector.isPressed() == false) {
            forward(.5);
        }
        forward(.9);
        idle();
        sleep(200);
        reverse(.9);
        idle();
        sleep(200);
        halt();
        verifyRed();

            /*forward(.1);
            sleep(1000);
            reverse(.1);
            sleep(1000);
            halt();
            fire();
            sleep(2000); //change accordingly
            forward(.1);
            sleep(1500);
            verifyRed();*/


        halt();

    }
    private void pressRedNoFire() {
        halt();
        reverse(.5);
        idle();
        sleep(200);
        halt();
        double redColorLeft = robot.left_color_sensor.red();
        double blueColorLeft = robot.left_color_sensor.blue();
        if (redColorLeft > blueColorLeft) {
            halt();
            //fire();
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
        reverse(.5);
        sleep(200);

        while(Math.abs(robot.lineSensor.getRawLightDetected() - followingValue) < .004) {
            halt();
            driveForTime(-90, 3000);
        }
        while(Math.abs(robot.lineSensor.getRawLightDetected() - followingValue) > .004){
            robot.front_left_motor.setPower(1);
            robot.back_left_motor.setPower(-1);
            robot.front_right_motor.setPower(1);
            robot.back_right_motor.setPower(1);
        }
        //This technically works but if we have issues I can do a more advanced version which is better.
        /*double initialHeading = robot.gyro.getHeading();
        while (Math.abs(robot.lineSensor.getLightDetected() < followingValue + .003)) {
            if (initialHeading - robot.gyro.getHeading() > 5) {
                robot.back_left_motor.setPower(robot.back_left_motor.getPower() + .01);
                robot.back_right_motor.setPower(robot.back_right_motor.getPower() - .01);
            } else if (robot.gyro.getHeading() - initialHeading > 5) {
                robot.back_left_motor.setPower(robot.back_left_motor.getPower() - .01);
                robot.back_right_motor.setPower(robot.back_right_motor.getPower() + .01);
            }
        }*/

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

        while ((robot.lineSensor.getRawLightDetected() < .007) && opModeIsActive()) {
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
            leftSpeed = .3 - (zAccumulated - target) / 100;  //Calculate speed for each side
            rightSpeed = .3 + (zAccumulated - target) / 100;  //See Gyro Straight video for detailed explanation
            leftSpeed = Range.clip(leftSpeed, -1, 1);
            rightSpeed = Range.clip(rightSpeed, -1, 1);
            drive(leftSpeed, rightSpeed);
            zAccumulated = robot.gyro.getIntegratedZValue();
            telemetry.addData("Driving, ods value is", robot.lineSensor.getRawLightDetected());
            telemetry.update();
            idle();
        }

    }

    private void followLine() {
        double counter = 0;
        double currentLightDetected = robot.lineSensor.getRawLightDetected();
        correction = (whiteLightValue - robot.lineSensor.getRawLightDetected());
        robot.back_right_motor.setPower(0);
        robot.back_left_motor.setPower(0);
        robot.front_left_motor.setPower(0);
        robot.front_right_motor.setPower(0);
        idle();
        if (Math.abs(robot.lineSensor.getRawLightDetected()) < followingValue + .003 ){
            telemetry.addData("I found the Line", "");
            telemetry.update();
        }

        while (robot.wallDetector.isPressed() == false && opModeIsActive()) {
            correction = (followingValue - currentLightDetected);
            telemetry.update();
            if(correction <= 0) {
                leftSpeed = .5 - correction*4;
                rightSpeed = .5;
                drive(leftSpeed, rightSpeed);
            }
            if(correction > 0) {
                leftSpeed = .5 + correction*4;
                rightSpeed = .5;
                drive(leftSpeed, rightSpeed);
            }
            currentLightDetected = robot.lineSensor.getRawLightDetected();
            idle();
        }
        while(robot.wallDetector.isPressed() == true && opModeIsActive()){
            driveGyroStraight(-90, .3);
            sleep(100);
            halt();
            reverse(.2);
            sleep(300);
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
            if (zAccumulated < target) {  //if gyro is positive, we will turn right
                //leftSpeed = turnSpeed;
                //rightSpeed = -turnSpeed;
                drive(turnSpeed, -turnSpeed);

            }

            if (zAccumulated > target) {  //if gyro is positive, we will turn left
                //leftSpeed = -turnSpeed;
                // rightSpeed = turnSpeed;
                drive(-turnSpeed, turnSpeed);
            }
            idle();

            zAccumulated = robot.gyro.getIntegratedZValue();
            //Set variables to gyro readings
            telemetry.addData("accu ",  zAccumulated);
            telemetry.update();

        }
        halt();




    }

    private void fire() {
        while((robot.lineSensor.getRawLightDetected() > .007)){
            reverse(.5);
        }

        for (int i = 0; i < 1; i += .1) {
            robot.right_balllauncher.setPower(i);
            sleep(10);
        }

        robot.ball_feeder.setPosition(20/180);
        ElapsedTime runtime = new ElapsedTime();
        runtime.reset();
        double n = 60;
        for (int i = 0; (i < 1 && opModeIsActive()); i += .1) {
            robot.right_balllauncher.setPower(i);
            sleep(10);
            idle();
        }
        robot.ball_feeder.setPosition(47.0/180.0);
        sleep(50);
        robot.ball_feeder.setPosition(177.0/180.0);
        sleep(400);
        robot.ball_feeder.setPosition(20/180);
        robot.ball_feeder.setPosition(47.0/180.0);
        sleep(500);
        robot.ball_feeder.setPosition(177.0/180.0);
        for (int j = 1; (j > 0 && opModeIsActive()); j -= .1) {
            robot.right_balllauncher.setPower(j);
            sleep(10);
            idle();
        }
        return;


    }



}

