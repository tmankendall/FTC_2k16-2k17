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

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

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

@TeleOp(name="Robot_Arm", group="RobotArm")
//@Disabled
public class Robot_Arm extends OpMode{

    /* Declare OpMode members. */
    NeilPushbot robot       = new NeilPushbot(); // use the class created to define a Pushbot's hardware
    // could also use HardwarePushbotMatrix class.
    int currentSpeed;
    double currentPosition = 0.0;
    double B = .1;
    double Y = .5;
    int X = 1;
    //int maxSweeperSpeed = 100.0;
    int maxSweeperSpeed = 100;
    double notPushingButton = 30.0/180.0;
    double pushingButton = 120.0/180.0;
    int n = 0;
    double j = .5;
    double apple = .5;

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
        //robot.ball_feeder.setPosition(1);
        //robot.ForkliftGrabber.setPosition(0);
        // positional based driving
        //telemetry.addData("The joystick", gamepad2.left_stick_y);
        //telemetry.addData("The joystick x", gamepad2.left_stick_x);
        telemetry.update();
        //robot.left_color_sensor.setI2cAddress();
        //up = -1, right = +1, others opposite
        double frontPower = -gamepad1.left_stick_y;
        double rightPower = gamepad1.left_stick_x;


        if (gamepad1.x) {
            robot.ball_feeder.setPosition(90.0 / 180.0);//open claw
           // robot.back_right_motor.setPower(.5);
        }
        if (gamepad1.b) {
            robot.ball_feeder.setPosition(0.0 / 180.0);//c.0lose claw
           // robot.back_right_motor.setPower(-.5);
        }

        if (gamepad1.a){
            robot.ForkliftGrabber.setPosition(0.0/180.0);
            //robot.ArmRotation.setPower(.5);
        }

        if (gamepad1.y){
            robot.ForkliftGrabber.setPosition(180.0/180.0);
            //robot.ArmRotation.setPower(-.5);
        }

        if (gamepad1.right_bumper){
        robot.ForkliftGrabber.setPosition(140.0/180.0);

        }
         if (gamepad1.left_bumper){
            //robot.ArmRotation.setPosition(.4);
        }
        if (gamepad1.dpad_left){
            //robot.back_right_motor.setPower(.5);
        }
        if (gamepad1.dpad_right){
            //robot.back_right_motor.setPower(-.5);
        }

        if(gamepad1.x){
            robot.back_right_motor.setPower(0);
            //robot.ArmRotation.setPosition(0.5);
        }
        robot.back_right_motor.setPower(gamepad1.left_stick_y/5);
        if (gamepad1.dpad_right){
            robot.ArmRotation.setPower(-.05);
        }
        if (gamepad1.dpad_left){
            robot.ArmRotation.setPower(.08);
        }
        if (gamepad1.dpad_up){
            robot.back_right_motor.setPower(-.12);

        }
        if (gamepad1.dpad_down){
            robot.back_right_motor.setPower(.12);
        }
        //robot.ArmRotation.setPower(-gamepad1.left_stick_x);
        robot.ArmRotation.setPower(-gamepad1.right_stick_x);
        if (gamepad1.left_bumper){
            for(double i = 21.0; i<180.0; i+=.45){
                robot.ArmSweeper.setPosition(i/180.0);

            }


            for (double j = 180.0; j>21.0; j-=1){
                robot.ArmSweeper.setPosition(j/180.0);
            }
        }
        else{
            robot.ArmSweeper.setPosition(30.0/180.0);
        }

        if (gamepad1.right_trigger>0){
            for (double i = 30.0; i<180.0; i+=.05){
                robot.back_right_motor.setPower(-.2);
                robot.ArmRotation.setPower(.8);//negative extends, clockwise positive
            }
            for (double i = 30.0; i<180.0; i+=.05){
                robot.back_right_motor.setPower(.2);
                robot.ArmRotation.setPower(-.8);//negative extends, clockwise positive
            }

        }
    }
    @Override
    public void stop() {
    }

}
