package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.util.ElapsedTime;


import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class RangeSensorTest{


    /**
     * {@link SensorREV2mDistance} illustrates how to use the REV Robotics
     * Time-of-Flight Range Sensor.
     * <p>
     * The op mode assumes that the range sensor is configured with a name of "sensor_range".
     * <p>
     * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
     * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
     *
     * @see <a href="http://revrobotics.com">REV Robotics Web Page</a>
     */
    @TeleOp(name = "Sensor: REV2mDistance", group = "Sensor")
    //@Disabled
    public class SensorREV2mDistance extends LinearOpMode {

        private DistanceSensor sensorRange;
        private ElapsedTime runtime = new ElapsedTime();

        @Override
        public void runOpMode() {
            // you can use this as a regular DistanceSensor.
            sensorRange = hardwareMap.get(DistanceSensor.class, "distance_sensor");

            // you can also cast this to a Rev2mDistanceSensor if you want to use added
            // methods associated with the Rev2mDistanceSensor class.
            Rev2mDistanceSensor sensorTimeOfFlight = (Rev2mDistanceSensor) sensorRange;

            telemetry.addData("Status", "Initialized");
            telemetry.addData("Distance Sensor", sensorRange.getDistance(DistanceUnit.CM));
            telemetry.update();

            waitForStart();

            runtime.reset();
            while (opModeIsActive()) {
                // generic DistanceSensor methods.

                telemetry.addData("Distance Sensor", sensorRange.getDistance(DistanceUnit.CM));
                telemetry.addData("deviceName", sensorRange.getDeviceName());
                telemetry.addData("range", String.format("%.01f mm", sensorRange.getDistance(DistanceUnit.MM)));
                telemetry.addData("range", String.format("%.01f cm", sensorRange.getDistance(DistanceUnit.CM)));
                telemetry.addData("range", String.format("%.01f m", sensorRange.getDistance(DistanceUnit.METER)));
                telemetry.addData("range", String.format("%.01f in", sensorRange.getDistance(DistanceUnit.INCH)));

                // Rev2mDistanceSensor specific methods.
                telemetry.addData("ID", String.format("%x", sensorTimeOfFlight.getModelID()));
                telemetry.addData("did time out", Boolean.toString(sensorTimeOfFlight.didTimeoutOccur()));

                telemetry.update();

                idle();
            }

        }

    }
}