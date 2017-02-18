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

@Autonomous(name="lineFollow", group="Andrew")  // @Autonomous(...) is the other common choice
//@Disabled
public class lineFollow extends LinearOpMode {

    /* Declare OpMode members. */
    private ElapsedTime runtime = new ElapsedTime();
    NeilPushbot robot = new NeilPushbot();
    double standardLightValue;
    double whiteLightValue = .2;
    //<<<<<<< Updated upstream
    int allRed = 1000;
    double power;
    double angle = 35;
    //=======
    double followingValue = 0.023;
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
        robot.front_left_motor.setDirection(DcMotorSimple.Direction.REVERSE);
        robot.back_left_motor.setDirection(DcMotorSimple.Direction.REVERSE);
        robot.front_right_motor.setDirection(DcMotorSimple.Direction.FORWARD);
        robot.back_right_motor.setDirection(DcMotorSimple.Direction.FORWARD);
        waitForStart();
        runtime.reset();
        driveGyroStraight(0,.7);
        followLine();
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
                leftSpeed = .8 - correction*4;
                rightSpeed = .8;
                drive(leftSpeed, rightSpeed);
            }
            if(correction > 0) {
                leftSpeed = .8 + correction*4;
                rightSpeed = .8;
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

        while ((robot.lineSensor.getRawLightDetected() < 30) && opModeIsActive()) {
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
}