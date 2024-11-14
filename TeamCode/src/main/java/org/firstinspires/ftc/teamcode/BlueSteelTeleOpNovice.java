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

import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
 * This file contains a minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
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

@TeleOp(name="Blue Steel TeleOp Novice", group="Linear Opmode")
//@Disabled
public class BlueSteelTeleOpNovice extends LinearOpMode {

    // Declare OpMode members.
    Hardware11241 robot = new Hardware11241();   // Use Team 11241's hardware

    public ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Initialize the hardware variables.
        // * The init() method of the hardware class does all the work here
        robot.init(hardwareMap);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        double leftFrontPower = 0;
        double rightFrontPower = 0;
        double leftRearPower = 0;
        double rightRearPower = 0;

        while (opModeIsActive()) {

            // Setup a variable for each drive wheel to save power level for telemetry
            // POV Mode uses right stick to go forward/back, and side to side, left x stick to turn/spin.
            // - This uses basic math to combine motions and is easier to drive straight.
            double driveForward = gamepad1.left_stick_y/2;
            double driveSideways = gamepad1.left_stick_x/2;
            double turn = -gamepad1.right_stick_x/2;

            leftFrontPower = Range.clip(driveForward - driveSideways + turn, -1, 1);
            rightFrontPower = Range.clip(driveForward + driveSideways - turn, -1, 1);
            leftRearPower = Range.clip(driveForward + driveSideways + turn, -1, 1);
            rightRearPower = Range.clip(driveForward - driveSideways - turn, -1, 1);

            /*
            // Slows down movement when the right stick is in use, making it easier to turn
            if((turn == 0) && (driveForward + driveSideways !=0)){
                leftFrontPower = Range.clip(driveForward - driveSideways, -0.75, 0.75);
                rightFrontPower = Range.clip(driveForward + driveSideways, -0.75, 0.75);
                leftRearPower = Range.clip(driveForward + driveSideways, -0.75, 0.75);
                rightRearPower = Range.clip(driveForward - driveSideways, -0.75, 0.75);
            }else if((turn == 0) && (driveForward + driveSideways == 0)){
                leftFrontPower = 0;
                rightFrontPower = 0;
                leftRearPower = 0;
                rightRearPower = 0;
            }else{
                leftFrontPower = Range.clip(driveForward - driveSideways + turn, -0.5, 0.5);
                rightFrontPower = Range.clip(driveForward + driveSideways - turn, -0.5, 0.5);
                leftRearPower = Range.clip(driveForward + driveSideways + turn, -0.5, 0.5);
                rightRearPower = Range.clip(driveForward - driveSideways - turn, -0.5, 0.5);
            }
            */
            // D pad controls liftArm.
            if (gamepad1.dpad_up) {
                robot.liftMotor.setPower(-1);
            }
            if (gamepad1.dpad_down)
                robot.liftMotor.setPower(1);
            else
                robot.liftMotor.setPower(0);

            // right bumper controls intakeR and intakeL
            if (gamepad1.right_bumper) {
                robot.intakeR.setPosition(0.70);
            }
            else
                robot.intakeR.setPosition(0.50);

            if (gamepad1.right_bumper) {
                robot.intakeL.setPosition(0.0);
            }
            else
                robot.intakeL.setPosition(1.0);


            // left bumper controls intakeL.
            //if (gamepad1.left_bumper) {
            //robot.intakeL.setPosition(0.50);

            // ??? controls drop?
            if (gamepad1.a) {
                robot.drop.setPosition(0.27);
            }
            else
                robot.drop.setPosition(0.0);

            // ??? controls paperairplane.


            // Send calculated power to wheels

            robot.leftFrontDrive.setPower(leftFrontPower);
            robot.rightFrontDrive.setPower(rightFrontPower);
            robot.leftRearDrive.setPower(leftRearPower);
            robot.rightRearDrive.setPower(rightRearPower);

            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            // Get the latest position, which includes the x and y coordinates, plus the
            // heading angle
            SparkFunOTOS.Pose2D pos = robot.myOtos.getPosition();

            // Reset the tracking if the user requests it
            if (gamepad1.y) {
                robot.myOtos.resetTracking();
            }

            // Re-calibrate the IMU if the user requests it
            if (gamepad1.x) {
                robot.myOtos.calibrateImu();
            }

            // Inform user of available controls
            telemetry.addLine("Press Y (triangle) on Gamepad to reset tracking");
            telemetry.addLine("Press X (square) on Gamepad to calibrate the IMU");
            telemetry.addLine();

            // Log the position to the telemetry
            telemetry.addData("X coordinate", pos.x);
            telemetry.addData("Y coordinate", pos.y);
            telemetry.addData("Heading angle", pos.h);

            // Update the telemetry on the driver station
            telemetry.update();
            //telemetry.addData("Motors", "LF (%.2f), RF (%.2f), LR (%.2f), RR (%.2f)", leftFrontPower, rightFrontPower, leftRearPower, rightRearPower);
            telemetry.update();
        }
    }
    private void configureOtos() {
        telemetry.addLine("Configuring OTOS...");
        telemetry.update();

        // Set the desired units for linear and angular measurements. Can be either
        // meters or inches for linear, and radians or degrees for angular. If not
        // set, the default is inches and degrees. Note that this setting is not
        // persisted in the sensor, so you need to set at the start of all your
        // OpModes if using the non-default value.
        // myOtos.setLinearUnit(DistanceUnit.METER);
        robot.myOtos.setLinearUnit(DistanceUnit.INCH);
        // myOtos.setAngularUnit(AnguleUnit.RADIANS);
        robot.myOtos.setAngularUnit(AngleUnit.DEGREES);

        // Assuming you've mounted your sensor to a robot and it's not centered,
        // you can specify the offset for the sensor relative to the center of the
        // robot. The units default to inches and degrees, but if you want to use
        // different units, specify them before setting the offset! Note that as of
        // firmware version 1.0, these values will be lost after a power cycle, so
        // you will need to set them each time you power up the sensor. For example, if
        // the sensor is mounted 5 inches to the left (negative X) and 10 inches
        // forward (positive Y) of the center of the robot, and mounted 90 degrees
        // clockwise (negative rotation) from the robot's orientation, the offset
        // would be {-5, 10, -90}. These can be any value, even the angle can be
        // tweaked slightly to compensate for imperfect mounting (eg. 1.3 degrees).
        SparkFunOTOS.Pose2D offset = new SparkFunOTOS.Pose2D(0, 0, 0);
        robot.myOtos.setOffset(offset);

        // Here we can set the linear and angular scalars, which can compensate for
        // scaling issues with the sensor measurements. Note that as of firmware
        // version 1.0, these values will be lost after a power cycle, so you will
        // need to set them each time you power up the sensor. They can be any value
        // from 0.872 to 1.127 in increments of 0.001 (0.1%). It is recommended to
        // first set both scalars to 1.0, then calibrate the angular scalar, then
        // the linear scalar. To calibrate the angular scalar, spin the robot by
        // multiple rotations (eg. 10) to get a precise error, then set the scalar
        // to the inverse of the error. Remember that the angle wraps from -180 to
        // 180 degrees, so for example, if after 10 rotations counterclockwise
        // (positive rotation), the sensor reports -15 degrees, the required scalar
        // would be 3600/3585 = 1.004. To calibrate the linear scalar, move the
        // robot a known distance and measure the error; do this multiple times at
        // multiple speeds to get an average, then set the linear scalar to the
        // inverse of the error. For example, if you move the robot 100 inches and
        // the sensor reports 103 inches, set the linear scalar to 100/103 = 0.971
        robot.myOtos.setLinearScalar(1.0);
        robot.myOtos.setAngularScalar(1.0);

        // The IMU on the OTOS includes a gyroscope and accelerometer, which could
        // have an offset. Note that as of firmware version 1.0, the calibration
        // will be lost after a power cycle; the OTOS performs a quick calibration
        // when it powers up, but it is recommended to perform a more thorough
        // calibration at the start of all your OpModes. Note that the sensor must
        // be completely stationary and flat during calibration! When calling
        // calibrateImu(), you can specify the number of samples to take and whether
        // to wait until the calibration is complete. If no parameters are provided,
        // it will take 255 samples and wait until done; each sample takes about
        // 2.4ms, so about 612ms total
        robot.myOtos.calibrateImu();

        // Reset the tracking algorithm - this resets the position to the origin,
        // but can also be used to recover from some rare tracking errors
        robot.myOtos.resetTracking();

        // After resetting the tracking, the OTOS will report that the robot is at
        // the origin. If your robot does not start at the origin, or you have
        // another source of location information (eg. vision odometry), you can set
        // the OTOS location to match and it will continue to track from there.
        SparkFunOTOS.Pose2D currentPosition = new SparkFunOTOS.Pose2D(0, 0, 0);
        robot.myOtos.setPosition(currentPosition);

        // Get the hardware and firmware version
        SparkFunOTOS.Version hwVersion = new SparkFunOTOS.Version();
        SparkFunOTOS.Version fwVersion = new SparkFunOTOS.Version();
        robot.myOtos.getVersionInfo(hwVersion, fwVersion);

        telemetry.addLine("OTOS configured! Press start to get position data!");
        telemetry.addLine();
        telemetry.addLine(String.format("OTOS Hardware Version: v%d.%d", hwVersion.major, hwVersion.minor));
        telemetry.addLine(String.format("OTOS Firmware Version: v%d.%d", fwVersion.major, fwVersion.minor));
        telemetry.update();
    }
}

