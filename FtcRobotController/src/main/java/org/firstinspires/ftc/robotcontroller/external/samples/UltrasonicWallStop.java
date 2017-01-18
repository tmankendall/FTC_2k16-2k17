package org.firstinspires.ftc.robotcontroller.external.samples;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

/**
 * Created by taylo on 1/17/2017.
 */
@Autonomous(name = "Sensor: UltrasonicWallStop", group = "Sensor")

public class UltrasonicWallStop extends LinearOpMode{

    OpticalDistanceSensor ODS;

    DcMotor leftMotor;
    DcMotor rightMotor;

    static double odsReadingRaw;
    static double odsReadingLinear;
    static double distanceFromWall = 1;
    static int maxSpeed = 15;

    @Override
    public void runOpMode()
    {
        ODS = hardwareMap.opticalDistanceSensor.get("ods");
        leftMotor = hardwareMap.dcMotor.get("leftMotor");
        rightMotor = hardwareMap.dcMotor.get("rightMotor");
        rightMotor.setDirection(DcMotor.Direction.REVERSE);

        rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        waitForStart();
// All that is left for now is to configure the triggers for the start and end of this opmode.
// An improvement to the program would be to take an average of the past xxx readings to determine the speed of motors
        while(opModeIsActive())
        {

            odsReadingRaw = ODS.getRawLightDetected();
            odsReadingLinear = Math.pow(odsReadingRaw, -0.5);

            if (odsReadingLinear > maxSpeed)
            {
                rightMotor.setPower(maxSpeed);
                leftMotor.setPower(maxSpeed);
            }
            else {
                rightMotor.setPower(odsReadingLinear - distanceFromWall);
                leftMotor.setPower(odsReadingLinear - distanceFromWall);
            }
        }
    }

}
