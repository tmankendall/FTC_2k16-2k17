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
        robot.front_left_motor.setDirection(DcMotorSimple.Direction.REVERSE);
        robot.back_left_motor.setDirection(DcMotorSimple.Direction.REVERSE);
        robot.front_right_motor.setDirection(DcMotorSimple.Direction.FORWARD);
        robot.back_right_motor.setDirection(DcMotorSimple.Direction.FORWARD);
        waitForStart();
        runtime.reset();
        followLine();
    }

    private void followLine() {
        double currentLightDetected = robot.lineSensor.getLightDetected();
        correction = (whiteLightValue - robot.lineSensor.getLightDetected());
        while (Math.abs(currentLightDetected - standardLightValue) < .1 * standardLightValue) {
            idle();
        }
        robot.back_right_motor.setPower(0);
        robot.back_left_motor.setPower(0);
        robot.front_left_motor.setPower(0);
        robot.front_right_motor.setPower(0);
        idle();
        while (robot.wallDetector.isPressed() == false) {

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
            telemetry.addData(">", robot.lineSensor.getLightDetected());
            telemetry.update();
        }
    }

    private void drive(double left, double right) {
        robot.front_left_motor.setPower(left);
        robot.front_right_motor.setPower(right);
        robot.back_left_motor.setPower(left);
        robot.back_right_motor.setPower(right);
    }
}