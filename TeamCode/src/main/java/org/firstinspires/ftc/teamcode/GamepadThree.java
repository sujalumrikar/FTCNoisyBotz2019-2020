/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import android.media.AudioManager;
import android.media.ToneGenerator;
import android.os.Handler;
import android.os.Looper;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;


/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="GamepadThree", group="Linear Opmode")
//@Disabled
public class GamepadThree extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    //private DcMotor leftDrive = null;
    //private DcMotor rightDrive = null;

    DcMotor leftDrive_1;
    DcMotor rightDrive_1;
    //DcMotor leftDrive_2;
    //DcMotor rightDrive_2;

    Servo servoMotor;
    DistanceSensor sensorRange;
    ColorSensor colorSensor;

    long mLastBeepMS = System.currentTimeMillis();
    long mGapMS = 1000;

    long getSleepDurationMS(int distanceCM){
        return (long) (distanceCM*distanceCM/40);
    }

    ToneGenerator toneG = new ToneGenerator(AudioManager.STREAM_ALARM, 100);

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        //Commented out the second drives since we're not using mechanum wheels right now
        leftDrive_1  = hardwareMap.dcMotor.get("leftDrive_1");
        rightDrive_1  = hardwareMap.dcMotor.get("rightDrive_1");
        //leftDrive_2  = hardwareMap.dcMotor.get("leftDrive_2");
        //rightDrive_2 = hardwareMap.dcMotor.get("rightDrive_2");


        servoMotor  = hardwareMap.servo.get("servo_motor_zero");
        sensorRange = hardwareMap.get(DistanceSensor.class, "distance_sensor");
        colorSensor = hardwareMap.get(ColorSensor.class, "color_sensor");

        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        leftDrive_1.setDirection(DcMotor.Direction.REVERSE);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            boolean turningAround = false;
            boolean slowMode = false;

            float turnSpeed = 1;
            double speed = 1;

            telemetry.addData("Distance", sensorRange.getDistance(DistanceUnit.CM));

            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            //telemetry.addData("Motors", "left (%.2f), right (%.2f)", leftPower, rightPower);
            telemetry.addData("Red", "Red:" + colorSensor.red());
            telemetry.addData("Blue", "Blue:" + colorSensor.blue());
            telemetry.addData("Green", "Green:" + colorSensor.green());
            telemetry.update();

            if(turningAround == false) {
                leftDrive_1.setPower((-gamepad1.left_stick_y) / 2);
                rightDrive_1.setPower((-gamepad1.right_stick_y) / 2);
                //leftDrive_2.setPower((-gamepad1.left_stick_y) / 2);
                //rightDrive_2.setPower((-gamepad1.right_stick_y) / 2);


            } else if(turningAround == true) {
                leftDrive_1.setPower((turnSpeed) / 1.25);
                rightDrive_1.setPower((-turnSpeed) / 1.25);
                //leftDrive_2.setPower((turnSpeed) / 1.25);
                //rightDrive_2.setPower((-turnSpeed) / 1.25);
            }

            if (gamepad2.a == true) {
                servoMotor.setPosition(180);
            }
            if (gamepad2.b == true) {
                servoMotor.setPosition(0);
            }

            if(gamepad1.x == true) {
                if(slowMode == false) {
                    slowMode = true;
                } else if(slowMode == true) slowMode = false;
                sleep(100);
            }

            if(slowMode==true){
                speed=.2;
            } else if (slowMode==false){
                speed = 1;
            }

            //turbo boost
            while(gamepad1.right_bumper == true && gamepad1.left_bumper == true && turningAround == false) {
                leftDrive_1.setPower(-gamepad1.left_stick_y);
                rightDrive_1.setPower(-gamepad1.right_stick_y);
                //leftDrive_2.setPower(-gamepad1.left_stick_y);
                //rightDrive_2.setPower(-gamepad1.right_stick_y);


            }

            while(gamepad1.dpad_up) {
                leftDrive_1.setPower(speed);
                //leftDrive_2.setPower(1);
                rightDrive_1.setPower(speed);
                //rightDrive_2.setPower(1);
            }

            while(gamepad1.dpad_down) {
                leftDrive_1.setPower(-speed);
                rightDrive_1.setPower(-speed);
                //leftDrive_2.setPower(-1);
                //rightDrive_2.setPower(-1);
            }

            while(gamepad1.dpad_left == true && turningAround == false) {
                leftDrive_1.setPower(-speed*.8);
                //leftDrive_2.setPower(0.8);
                rightDrive_1.setPower(speed*.8);
                //rightDrive_2.setPower(-0.8);
            }

            while(gamepad1.dpad_right == true && turningAround == false) {
                leftDrive_1.setPower(speed*.8);
                //leftDrive_2.setPower(-0.8);
                rightDrive_1.setPower(-speed*.8);
                //rightDrive_2.setPower(0.8);
            }

            int distanceEquation = (int)sensorRange.getDistance(DistanceUnit.CM);
            colorSensor.enableLed(false);
            if(distanceEquation > 0) {
                colorSensor.enableLed(true);

                if((colorSensor.red()>=115 && colorSensor.red()<=240) &&
                   (colorSensor.blue()>=110 && colorSensor.blue()<=135) &&
                   (colorSensor.green()>=215 && colorSensor.green()<=400)){
                mGapMS = getSleepDurationMS(distanceEquation);
                if(System.currentTimeMillis()>(mLastBeepMS+mGapMS)) {
                    toneG.startTone(ToneGenerator.TONE_CDMA_ALERT_CALL_GUARD, 200);
                    sleep(distanceEquation * distanceEquation / 80);
                   toneG.stopTone();
                    mLastBeepMS = System.currentTimeMillis();
                }
                }
            }

            if (gamepad1.b == true && turningAround == false) {
//                turningAround = true; //refer to lines 93-96
                leftDrive_1.setPower((turnSpeed) / 1.25);
                rightDrive_1.setPower((-turnSpeed) / 1.25);
                sleep(625);
//                turningAround = false;
            }

            // Setup a variable for each drive wheel to save power level for telemetry
            /*double leftPower;
            double rightPower;

            // Choose to drive using either Tank Mode, or POV Mode
            // Comment out the method that's not used.  The default below is POV.

            // POV Mode uses left stick to go forward, and right stick to turn.
            // - This uses basic math to combine motions and is easier to drive straight.
            double drive = -gamepad1.left_stick_y;
            double turn  =  gamepad1.right_stick_x;
            leftPower    = Range.clip(drive + turn, -1.0, 1.0) ;
            rightPower   = Range.clip(drive - turn, -1.0, 1.0) ;

            // Tank Mode uses one stick to control each wheel.
            // - This requires no math, but it is hard to drive forward slowly and keep straight.
            // leftPower  = -gamepad1.left_stick_y ;
            // rightPower = -gamepad1.right_stick_y ;

            // Send calculated power to wheels
            leftDrive.setPower(leftPower);
            rightDrive.setPower(rightPower);*/

            idle();

        }
    }
}
