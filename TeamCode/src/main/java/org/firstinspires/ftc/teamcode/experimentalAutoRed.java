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

@Autonomous(name="experimentalAutoRed", group="Andrew")  // @Autonomous(...) is the other common choice
//@Disabled
public class experimentalAutoRed extends LinearOpMode {

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
        robot.ball_feeder.setPosition(47.0/180.0);
        robot.ForkliftGrabber.setPosition(1);

   //     driveForTimeLeft(0, 5000);
            fire();
            driveForTime(0, 500);
            GyroTurn(48);
            telemetry.addLine("driving no value");
            telemetry.update();
//        driveNoLine(-35, .5);
            halt();
            driveGyroStraight(48, .2);
//        driveLine(-90, .5);
            if (isStopRequested()) {
                halt();
                stop();
            }
            GyroTurn(75);
            //driveGyroStraight(35, .3);
            if (isStopRequested() == false) {
                followLine();
            }
            //driveForTime(-90, 1000);
            //halt();
            //ColorAlignRed();
            //pressRed();
            //fire();
            //sleep(7000);
            //followLine();
            if (isStopRequested() == false) {
                verifyRed();
            }
            //fire();
            //ColorAlignRed();
            //pressRed();
        if(isStopRequested() == false) {


            while (robot.wallDetector.isPressed() == false) {
                driveGyroStraight(-90, -1);

            }
            if (robot.wallDetector.isPressed() == true) {
                drive(-1, -1);
                sleep(1500);
                halt();
            }
        }

        if(isStopRequested() == true){
            halt();
        }

    }
//    private void driveNoLine(int angle, double power){
//        if (currentValue<followingValue) {
//            driveGyroStraight(angle, power);
//            currentValue = robot.lineSensor.getRawLightDetected();
//            telemetry.addData("driving", "driving gyro");
//            telemetry.update();
//        }
//    }
//    private void driveLine(int angle, double power){
//        GyroTurn(angle);
//        if (robot.wallDetector.isPressed() == false){
//            driveGyroStraight(angle, power);
//
//        }
//        if (robot.wallDetector.isPressed()==true){
//            halt();
//        }
//    }
    private void getColors(){
        getRightValueRed();
        getLeftValueBlue();
        getLeftValueRed();
        getRightValueBlue();
    }
    private void getRightValueRed(){
        robot.joshThisIsForYou.setDigitalChannelState(3, true);
        robot.joshThisIsForYou.setDigitalChannelState(4, false);
        //redColorRight = robot.right_color_sensor.red();
        telemetry.addData("right red value is ", redColorRight);
        telemetry.update();

    }
    private void getLeftValueRed(){
        robot.joshThisIsForYou.setDigitalChannelState(4, true);
        robot.joshThisIsForYou.setDigitalChannelState(3, false);
        //redColorLeft = robot.right_color_sensor.red();
        telemetry.addData("left red value is ", redColorLeft);
        telemetry.update();

    }
    private void getRightValueBlue(){
        robot.joshThisIsForYou.setDigitalChannelState(3, true);
        robot.joshThisIsForYou.setDigitalChannelState(4, false);
        //blueColorRight = robot.right_color_sensor.blue();
        telemetry.addData("right blue value is ", blueColorRight);
        telemetry.update();
    }
    private void getLeftValueBlue(){
        robot.joshThisIsForYou.setDigitalChannelState(4, true);
        robot.joshThisIsForYou.setDigitalChannelState(3, false);
        //blueColorLeft = robot.right_color_sensor.blue();
        telemetry.addData("left blue value is ", blueColorLeft);
        telemetry.update();
    }
    private void getBlue(){
        blueColor = robot.right_color_sensor.blue();

    }
    private void getRed(){
        redColor = robot.right_color_sensor.red();
    }
//    private void ColorAlignRed(){
//        getColors();
//        if (redColorRight > blueColorRight){
//            while(blueColorLeft>redColorLeft && opModeIsActive()){
//                robot.back_left_motor.setPower(-1);
//                robot.front_right_motor.setPower(-1);
//                robot.back_right_motor.setPower(1);
//                robot.front_left_motor.setPower(1);
//            }
//
//        }
//        if (redColorLeft > blueColorLeft){
//            while(blueColorRight> redColorRight && opModeIsActive()){
//                robot.back_left_motor.setPower(1);
//                robot.front_right_motor.setPower(1);
//                robot.back_right_motor.setPower(-1);
//                robot.front_left_motor.setPower(-1);
//            }
//        }
//
//
////    }
////    private void ColorConfirm() {
////
////        getColors();
////        if (redColorRight < blueColorRight && redColorLeft < blueColorLeft && opModeIsActive()) {
////            pressRed();
////            telemetry.addData("I found the red color!", "yay!");
////            telemetry.update();
////        }
////        Drive2ndBeacon();
////    }
    private void verifyRed(){
        getBlue();
        getRed();
        if (redColor>650){
            //fire();
            halt();
            telemetry.addData("I found the red color!", "yay!");
            telemetry.update();
        }
        else{
            telemetry.addLine("not red");
            telemetry.update();
            pressRed();

        }

    }

    private void pressRed() {
        halt();
        reverse(.1);
        idle();
        sleep(100);
        halt();
        //getColors();
        //fire();
        sleep(7000);
        verifyRed();
        sleep(1000);
        /*while (robot.wallDetector.isPressed() == false && opModeIsActive()) {
                forward(.5);
            }*/
        forward(.5);
        //idle();
        sleep(50);

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
//    private void pressRedNoFire() {
//        halt();
//        reverse(.5);
//        idle();
//        sleep(200);
//        halt();
//        double redColorLeft = robot.left_color_sensor.red();
//        double blueColorLeft = robot.left_color_sensor.blue();
//        if (redColorLeft > blueColorLeft) {
//            halt();
//            //fire();
//            Drive2ndBeacon();
//
//        } else if (blueColorLeft > redColorLeft) {
//            forward(.1);
//            sleep(1000);
//            reverse(.1);
//            sleep(1000);
//            halt();
//            //fire();
//            sleep(6000); //change accordingly
//            forward(.1);
//            sleep(1500);
//            verifyRed();
//
//        }
//        halt();
//
//    }
    /*private void nextBeacon() {
        reverse(.1);
        sleep(300);
        Drive2ndBeacon();
    }*/

//    private void Drive2ndBeacon() {
//        reverse(.5);
//        sleep(200);
//        while(Math.abs(robot.lineSensor.getRawLightDetected() - followingValue) > .007 && opModeIsActive()){
//            robot.front_left_motor.setPower(-1);
//            robot.back_left_motor.setPower(1);
//            robot.front_right_motor.setPower(1);
//            robot.back_right_motor.setPower(-1);
//        }
//        //This technically works but if we have issues I can do a more advanced version which is better.
//        /*double initialHeading = robot.gyro.getHeading();
//        while (Math.abs(robot.lineSensor.getLightDetected() < followingValue + .003)) {
//            if (initialHeading - robot.gyro.getHeading() > 5) {
//                robot.back_left_motor.setPower(robot.back_left_motor.getPower() + .01);
//                robot.back_right_motor.setPower(robot.back_right_motor.getPower() - .01);
//            } else if (robot.gyro.getHeading() - initialHeading > 5) {
//                robot.back_left_motor.setPower(robot.back_left_motor.getPower() - .01);
//                robot.back_right_motor.setPower(robot.back_right_motor.getPower() + .01);
//            }
//        }*/
//
//    }

//    private void goRight(int time) {
//        robot.back_left_motor.setPower(-1);
//        robot.front_right_motor.setPower(-1);
//        robot.back_right_motor.setPower(1);
//        robot.front_left_motor.setPower(1);
//        sleep(time);
//        robot.back_left_motor.setPower(0);
//        robot.front_right_motor.setPower(0);
//        robot.back_right_motor.setPower(0);
//        robot.front_left_motor.setPower(0);
//    }
//
//    private void goLeft(int time) {
//        robot.back_left_motor.setPower(1);
//        robot.front_right_motor.setPower(1);
//        robot.back_right_motor.setPower(-1);
//        robot.front_left_motor.setPower(-1);
//        sleep(time);
//        robot.back_left_motor.setPower(0);
//        robot.front_right_motor.setPower(0);
//        robot.back_right_motor.setPower(0);
//        robot.front_left_motor.setPower(0);
//    }

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

//    private void driveForTimeLeft(double angle, double time) {
//        double frontSpeed;
//        double backSpeed;
//        //double initialTime = runtime.milliseconds();
//        double initialTime = runtime.milliseconds();
//        double target = angle;  //Starting direction
//        zAccumulated = robot.gyro.getIntegratedZValue();
//        double currentTime = runtime.milliseconds();//Current direction
//        double desiredTime = time + currentTime;
//
//        while ((desiredTime-currentTime) > 0 && opModeIsActive()) {
//            currentTime = runtime.milliseconds();
//            frontSpeed = .3 - (zAccumulated - target) / 100.0;  //Calculate speed for each side
//            backSpeed = .3 + (zAccumulated - target) / 100.0;  //See Gyro Straight video for detailed explanation
//            backSpeed = Range.clip(backSpeed, -1, 1);
//            frontSpeed = Range.clip(frontSpeed, -1, 1);
//            driveLeft(frontSpeed, backSpeed);
//            zAccumulated = robot.gyro.getIntegratedZValue();
//            idle();
//        }
//
//    }

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
        double initialTime = getRuntime();

        while ((getRuntime() - initialTime < 700) && opModeIsActive()) {
            correction = (followingValue - currentLightDetected);
            telemetry.update();
            if(correction <= 0) {
                leftSpeed = .2;
                rightSpeed = .2 - correction*100.0;
                drive(leftSpeed, rightSpeed);
            }
            if(correction > 0) {
                leftSpeed = .2 + correction*100.0;
                rightSpeed = .2;
                drive(leftSpeed, rightSpeed);
            }
            telemetry.addData("Correction: ", correction);
            telemetry.update();
            currentLightDetected = robot.lineSensor.getRawLightDetected();
            idle();
        }
        if(robot.wallDetector.isPressed() == true && opModeIsActive()){
            halt();
        }


    }


    private void drive(double left, double right) {
        robot.front_left_motor.setPower(left);
        robot.front_right_motor.setPower(right);
        robot.back_left_motor.setPower(left);
        robot.back_right_motor.setPower(right);
    }

//    private void driveLeft(double front, double back) {
//        robot.front_left_motor.setPower(-front);
//        robot.front_right_motor.setPower(front);
//        robot.back_left_motor.setPower(back);
//        robot.back_right_motor.setPower(-back);
//        telemetry.addData("Front Speed", front);
//        telemetry.addData("Back Speed", back);
//        telemetry.addData("Front to back Ratio", front/back);
//        telemetry.update();
//        sleep(300);
//    }

    public void GyroTurn(int target) {
        zAccumulated = robot.gyro.getIntegratedZValue();  //Set variables to gyro readings
        double turnSpeed = 0.15;

        while (Math.abs(zAccumulated - target) > 2 && opModeIsActive()) {  //Continue while the robot direction is further than three degrees from the target
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
        sleep(200);

        // robot.ball_feeder.setPosition(20/180);
        robot.ball_feeder.setPosition(47.0/180.0);
        sleep(200);
        robot.ball_feeder.setPosition(177.0/180.0);
        sleep(200);
        for (float j = 1; (j > 0 && opModeIsActive()); j -= .01) {
            robot.right_balllauncher.setPower(j);
            sleep(20);
            idle();
        }
        return;


    }



    }

