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
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
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

@TeleOp(name="Template: Linear OpMode", group="Linear Opmode")  // @Autonomous(...) is the other common choice
//@Disabled
public class RobotArmCode extends LinearOpMode {

    /* Declare OpMode members. */
    private ElapsedTime runtime = new ElapsedTime();
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


    private double gPos = 0;
    private double WrPos = 0;
    private double in = 0;
    private double out = 0;
    private int delta = 10;
    // DcMotor leftMotor = null;
    // DcMotor rightMotor = null;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        RobotArmPushBot robot = new RobotArmPushBot();
        robot.init(hardwareMap);
        runtime.startTime();

        waitForStart();



        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.update();
            //GamePad 1 coarse
            telemetry.update();
            //robot.left_color_sensor.setI2cAddress();
            //up = -1, right = +1, others opposite
            double frontPower = -gamepad1.left_stick_y;
            double rightPower = gamepad1.left_stick_x;


            if (gamepad1.x) {
                robot.grabber.setPosition(90.0 / 180.0);//open claw
                // robot.back_right_motor.setPower(.5);
            }
            if (gamepad1.b) {
                robot.grabber.setPosition(0.0 / 180.0);//c.0lose claw
                // robot.back_right_motor.setPower(-.5);
            }

            if (gamepad1.a){
                robot.wrist.setPosition(0.0/180.0);
                //robot.ArmRotation.setPower(.5);
            }

            if (gamepad1.y){
                robot.wrist.setPosition(180.0/180.0);
                //robot.ArmRotation.setPower(-.5);
            }

            if (gamepad1.right_bumper){
                robot.wrist.setPosition(140.0/180.0);

            }
            if (gamepad1.dpad_right){
                robot.baseRotation.setPower(-.05);
            }
            if (gamepad1.dpad_left){
                robot.baseRotation.setPower(.08);
            }
            if (gamepad1.dpad_up){
                robot.extension.setPower(-.12);

            }
            if (gamepad1.dpad_down){
                robot.extension.setPower(.12);
            }
            //robot.ArmRotation.setPower(-gamepad1.left_stick_x);
            robot.baseRotation.setPower(-gamepad1.right_stick_x);
            if (gamepad1.left_bumper){
                for(double i = 21.0; i<180.0; i+=.45){
                    robot.sweeper.setPosition(i/180.0);

                }


                for (double j = 180.0; j>21.0; j-=1){
                    robot.sweeper.setPosition(j/180.0);
                }
            }
            else{
                robot.sweeper.setPosition(30.0/180.0);
            }

            if (gamepad1.right_trigger>0){
                for (double i = 30.0; i<180.0; i+=.05){
                    robot.extension.setPower(-.2);
                    robot.baseRotation.setPower(.8);//negative extends, clockwise positive
                    double runTime = getRuntime();
                    while(getRuntime() - runTime < 100 && opModeIsActive())
                    {
                        idle();
                    }
                }
                for (double i = 30.0; i<180.0; i+=.05){
                    robot.extension.setPower(.2);
                    robot.baseRotation.setPower(-.8);//negative extends, clockwise positive
                    double runTime = getRuntime();
                    while(getRuntime() - runTime < 100 && opModeIsActive())
                    {
                        idle();
                    }
                }

            }

            if(Math.abs(gamepad1.right_stick_x) > .1) {
                robot.baseRotation.setPower(gamepad1.right_stick_x);
            }
            else
            {
                robot.baseRotation.setPower(0);
            }

            if(Math.abs(gamepad1.left_stick_y) > .1) {
                robot.extension.setPower(gamepad1.left_stick_y);
            }
            else
            {
                robot.extension.setPower(0);
            }

            //GamePad 2 fine
            if(Math.abs(gamepad2.left_stick_y) > .1) {
                WrPos = WrPos + 5.0*gamepad2.right_stick_x;
                robot.wrist.setPosition(WrPos);
            }
            if(Math.abs(gamepad2.right_stick_x) > .1) {
                gPos = gPos + 10.0*gamepad2.right_stick_x;
                robot.grabber.setPosition(gPos);
            }

            if(gamepad2.a)
            {
                robot.sweeper.setPosition(out);
            }
            else
            {
                robot.sweeper.setPosition(in);
            }

            if(gamepad2.left_trigger > .1)
            {
                robot.baseRotation.setPower(.5*gamepad2.left_trigger);
            }
            else if(gamepad2.right_trigger > .1)
            {
                robot.baseRotation.setPower(-.5*gamepad2.right_trigger);
            }
            else if(Math.abs(gamepad1.right_stick_x) < .1)
            {
                robot.baseRotation.setPower(0);
            }

            if (gamepad2.dpad_up)
            {
                robot.extension.setPower(.1);
            }
            else if (gamepad2.dpad_down)
            {
                robot.extension.setPower(-.1);
            }
            else if (gamepad2.dpad_left || gamepad2.dpad_right)
            {
                robot.extension.setPower(0);
            }

            // eg: Run wheels in tank mode (note: The joystick goes negative when pushed forwards)
            // leftMotor.setPower(-gamepad1.left_stick_y);
            // rightMotor.setPower(-gamepad1.right_stick_y);
        }
    }
}
