

package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Created by taylo on 1/17/2017.
 */
@Autonomous(name = "Sensor: UltrasonicWallStop", group = "Sensor")

public class UltrasonicWallStop extends LinearOpMode{

    OpticalDistanceSensor ODS;
    private ElapsedTime runtime = new ElapsedTime();
    NeilPushbot robot = new NeilPushbot();

    static double odsReadingRaw;
    static double odsReadingLinear;
    static double distanceFromWall = 1;
    static double average = 0;
    static int counter = 0;
    static double totalReads = 0;
    double[] pastReadings = new double[5];

    @Override
    public void runOpMode()
    {
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        robot.init(hardwareMap);

        waitForStart();
// All that is left for now is to configure the triggers for the start and end of this opmode.
// An improvement to the program would be to take an average of the past xxx readings to determine the speed of motors
        while(opModeIsActive())
        {

            odsReadingRaw = ODS.getRawLightDetected();
            odsReadingLinear = Math.pow(odsReadingRaw, -0.5);
            if (counter < 5)
            {
                counter++;
            }
            if (counter > 1)
            {
                for (int i = counter; i > 1; i++) {
                    pastReadings[counter - 1] = pastReadings[counter - 2];
                }
            }
            pastReadings[0] = odsReadingLinear;
            for (int i = 0; i < counter; i++)
            {
                totalReads += pastReadings[i];
            }
            average = (totalReads/counter);
            if (average > distanceFromWall) {
                robot.front_right_motor.setPower(-1);
//            robot.front_left_motor.setPower(1);
//            robot.back_right_motor.setPower(1);
//            robot.back_left_motor.setPower(-1);
            }
            else {
            }
        }
    }

}
