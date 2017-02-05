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

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcontroller.external.samples.HardwarePushbot;

/**
 * This file provides basic Telop driving for a Pushbot robot.
 * The code is structured as an Iterative OpMode
 *
 * This OpMode uses the common Pushbot hardware class to define the devices on the robot.
 * All device access is managed through the HardwarePushbot class.
 *
 * This particular OpMode executes a basic Tank Drive Teleop for a PushBot
 * It raises and lowers the claw using the Gampad Y and A buttons respectively.
 * It also opens and closes the claws slowly using the left and right Bumper buttons.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="teleop mecanum", group="Andrew")
//@Disabled
public class Teleopmecanum extends OpMode{

    /* Declare OpMode members. */
    NeilPushbot robot       = new NeilPushbot(); // use the class created to define a Pushbot's hardware
    // could also use HardwarePushbotMatrix class.
    int currentSpeed;
    double B = .1;
    double Y = .5;
    int X = 1;
    int maxSweeperSpeed = 100;
    double notPushingButton = 30.0/180.0;
    double pushingButton = 120.0/180.0;
    int n = 0;
    double j = .5;
    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Say", "Hello Driver");
        telemetry.addData("Say", "Remember the driving contorls\nDirectional left joystick, turn angle with right\nRight Trigger is the pulley\n X, B, and Y are speeds for the launcher\n");
        telemetry.update();
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {

    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {

    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        double left;
        double right;
        double threshold = .02;

        // positional based driving
        telemetry.addData("The joystick", gamepad2.left_stick_y);
        telemetry.addData("The joystick x", gamepad2.left_stick_x);
        telemetry.update();

        //up = -1, right = +1, others opposite
        if(java.lang.Math.abs(gamepad1.left_stick_y) > threshold || java.lang.Math.abs(gamepad1.left_stick_x) > threshold)
        {
            robot.front_right_motor.setPower(((gamepad1.left_stick_y + gamepad1.left_stick_x)*(65.0/152.0))*.65);
            robot.front_left_motor.setPower(((-gamepad1.left_stick_y + gamepad1.left_stick_x)*(65.0/152.0))*.65);
            robot.back_right_motor.setPower((-gamepad1.left_stick_y + gamepad1.left_stick_x)*.65);
            robot.back_left_motor.setPower((gamepad1.left_stick_y + gamepad1.left_stick_x)*.65);
        }
        else if (java.lang.Math.abs(gamepad1.right_stick_x) < threshold)
        {
            robot.front_right_motor.setPower(0);
            robot.front_left_motor.setPower(0);
            robot.back_left_motor.setPower(0);
            robot.back_right_motor.setPower(0);
        }
//        if (java.lang.Math.abs(gamepad1.left_stick_y) > threshold)
//        {
//            robot.front_right_motor.setPower(gamepad1.left_stick_y/.5);
//            robot.front_left_motor.setPower(-gamepad1.left_stick_y/.5);
//            robot.back_right_motor.setPower(-gamepad1.left_stick_y/.5);
//            robot.back_left_motor.setPower(gamepad1.left_stick_y/.5);
//        }
//        if (java.lang.Math.abs(gamepad1.left_stick_y) > threshold)
//        {
//            robot.front_right_motor.setPower(1);
//            robot.front_left_motor.setPower(-1);
//            robot.back_right_motor.setPower(-1);
//            robot.back_left_motor.setPower(1);
//        }
//        if (gamepad1.left_stick_y < threshold)
//        {
//            robot.front_right_motor.setPower(-1);
//            robot.front_left_motor.setPower(1);
//            robot.back_right_motor.setPower(1);
//            robot.back_left_motor.setPower(-1);
//        }
        if (java.lang.Math.abs(gamepad1.right_stick_x) > threshold)
        {
            robot.front_right_motor.setPower((robot.front_right_motor.getPower() + gamepad1.right_stick_x)*2);
            robot.front_left_motor.setPower((robot.front_left_motor.getPower() + gamepad1.right_stick_x)*2);
            robot.back_right_motor.setPower((robot.back_right_motor.getPower() - gamepad1.right_stick_x)*2);
            robot.back_left_motor.setPower((robot.back_left_motor.getPower() - gamepad1.right_stick_x)*2);
        }

//        // Use gamepad left & right Bumpers to open and close the claw
//        if (gamepad1.b) {
//            robot.right_balllauncher.setPower(B);
//        }
//        else if (gamepad1.y) {
//            robot.right_balllauncher.setPower(Y);
//        }
//        else if (gamepad1.x) {
//            robot.right_balllauncher.setPower(X);
//        }
        robot.right_balllauncher.setPower(-gamepad2.right_trigger);
        if(gamepad2.dpad_left)
        {
            robot.forklift.setPower(1);
        }
        else if(gamepad2.dpad_right)
        {
            robot.forklift.setPower(0);
        }
        if(gamepad2.left_bumper)
        {
            robot.ball_feeder.setPosition(.3);
        }
        if(gamepad2.right_bumper)
        {
            robot.ball_feeder.setPosition(90.0/180.0 + n/180.0 );
        }
        //if(gamepad1.a)
        //{
        //    fire();
        //}
        if (gamepad1.dpad_up){
            robot.button_pusher.setPosition(pushingButton);
        }

        if(gamepad1.dpad_down){
            robot.button_pusher.setPosition(notPushingButton);
        }

        if (gamepad2.y) {
            robot.forklift.setPower(1);
        }

        if (gamepad2.b){
            robot.forklift.setPower(0);
        }

        if (gamepad2.dpad_up) {
            telemetry.clear();
            n = n + 1;
            telemetry.addData("Upper Angle ", n + 90);
            telemetry.update();

        }
        if (gamepad2.dpad_down){
            telemetry.clear();
            n = n - 1;
            telemetry.addData("Upper Angle" , n + 90);
            telemetry.update();
        }

        if (gamepad2.left_stick_y != 0)
        {
            robot.forkliftGrabber.setPower(gamepad2.left_stick_y);
        }
        if (gamepad2.right_stick_x > 0)
        {
            if (j < .86) {
                j = j + .01;
            }
            robot.button_pusher.setPosition(j);
        }
        if (gamepad2.right_stick_x < 0)
        {
            if (j > .15) {
                j = j - .01;
            }
            robot.button_pusher.setPosition(j);
        }

    }

    private void fire()
    {
        ElapsedTime runtime = new ElapsedTime();
        runtime.reset();

        for(int i = 0; i < 1; i += .01)
        {
            robot.right_balllauncher.setPower(i);
        }
        robot.ball_feeder.setPosition(120.0/180.0 + n/180.0);
        robot.ball_feeder.setPosition(.5);
        robot.right_balllauncher.setPower(0);
        return;


    }
    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }

}
