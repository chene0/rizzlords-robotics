/*
 * Copyright (c) 2021 OpenFTC Team
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

package org.firstinspires.ftc.teamcode;

import android.graphics.Color;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.RobotLog;
import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.robotcore.external.navigation.Rotation;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

import java.util.ArrayList;
import java.util.List;

// This part slightly changed due to having to define this class as autonomous.
@Autonomous(name = "LEFT SIDE Rizzlords Autonomous", group = "Rizzlords")
public class Rizzlords_Autonomous extends LinearOpMode {

    private DcMotor TopRight;
    private DcMotor BottomRight;
    private DcMotor TopLeft;
    private DcMotor BottomLeft;

    private DcMotor Arm;
    private DcMotor ForeArm;
    private Servo Hand;
    private Servo CameraServo;

    private double CurrRotation;

    private ElapsedTime runtime = new ElapsedTime();
    private double encoderPower;
    private int armEncoder;
    private int foreArmEncoder;
    private double handOpen;
    private double handClosed;





    /**
     * This function is executed when this Op Mode is selected from the Driver Station.
     */
    @Override
    public void runOpMode() {
        telemetry.setMsTransmissionInterval(50);
        TopRight = hardwareMap.get(DcMotor.class, "Top Right");
        BottomRight = hardwareMap.get(DcMotor.class, "Bottom Right");
        TopLeft = hardwareMap.get(DcMotor.class, "Top Left");
        BottomLeft = hardwareMap.get(DcMotor.class, "Bottom Left");

        Arm = hardwareMap.get(DcMotor.class, "Arm1");
        ForeArm = hardwareMap.get(DcMotor.class, "Forearm");
        Hand = hardwareMap.get(Servo.class, "Hand");
        CameraServo = hardwareMap.get(Servo.class, "CameraServo");
        encoderPower = .3;
        handOpen = 1;
        handClosed = 0.7;
        RunUsingEncoder(Arm);
        RunUsingEncoder(ForeArm);

        CurrRotation = 0;

        BottomLeft.setDirection(DcMotorSimple.Direction.REVERSE);
//        TopLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        TopRight.setDirection(DcMotorSimple.Direction.REVERSE);
        HandControl();

        // Wait for the DS start button to be touched.
        telemetry.addData("DS preview on/off", "3 dots, Camera Stream");
        telemetry.addData(">", "Touch Play to start OpMode");
        telemetry.update();

        /* Actually do something useful */
        waitForStart();
        // Unlike TeleOp, Autonomous just calls a sequence of commands that do not repeat, which in this case
        // was a function containing that sequence.
        RunSequence();

        telemetry.update();
    }

    private void RunUsingEncoder(DcMotor motor) {
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor.setTargetPosition(0);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor.setPower(.1);
    }

    /**
     * the sequence of movements to move to a medium/small pole and place cone
     * <p>
     * trajectory arguments meaning
     * 0 - default trajectory
     * 1 - end left
     * 2 - end middle
     * 3 - end right
     */
    private void RunSequence() {
        // This travel function was how I defined the robot's omnidirection movement using input values.
        // In this case the arguments where horizontal power, vertical power, rotational power, duration.
        Travel(0.3, 0, 0, 1.5);

        Travel(0, -0.3, 0, 3.5);

        // Setting the arm preset was similar to the method described in the teleop class except 
        // instead of checking for inputs from the controller the function accepts an argument
        // as an integer and sets the robot arm position based on that input.
        FullArmPreset(2);

        // This runtime object will allow you to provide timing for your autonomous code.
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 5)) {
            telemetry.addData("Path", "Leg 1: %4.1f S Elapsed", runtime.seconds());
            telemetry.update();
        }

        HandControl();
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 1)) {
            telemetry.addData("Path", "Leg 1: %4.1f S Elapsed", runtime.seconds());
            telemetry.update();
        }

        Travel(0, 0.1, 0, 4);

        FullArmPreset(0);

        Travel(0, -0.1, 0, 4);

        TerminateMovement();
    }

    private void FullArmPreset(int preset){
        ArmControlPreset(preset);
        ForeArmControlPreset(preset);
    }

    private void Travel(double xVal, double yVal, double rVal, double time){
        moveXY(xVal, yVal, rVal);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < time)) {
            telemetry.addData("Path", "Leg 1: %4.1f S Elapsed", runtime.seconds());
            telemetry.update();
        }
        TerminateMovement();
    }

    /**
     * terminates movement
     */
    private void TerminateMovement() {
        moveXY(0, 0, 0);
        sleep(100);
    }

    /**
     * arm control specific
     */
//    private void ArmControl(int target) {
//        arm.setPower(1);
//        if (target < 0){
//            target = 0;
//        }
//        else if(target > 11416){
//            target = 11416;
//        }
//        arm.setTargetPosition(target);
//    }

    /**
     * @param position position = 0: default
     *                 position = 1: ready to pickup
     *                 position = 2: pickup
     *                 position = 3: place
     */
    private void ArmControlPreset(int position) {
        Arm.setPower(encoderPower);
        switch (position) {
            case 0:
                telemetry.addData("target pos", 0);
                telemetry.update();
                Arm.setTargetPosition(0);
                break;
            case 1:
                telemetry.addData("target pos", 884);
                telemetry.update();
                Arm.setTargetPosition(-67);
                break;
            case 2:
                telemetry.addData("target pos", 6736);
                telemetry.update();
                Arm.setTargetPosition(161);
                break;
            case 3:
                telemetry.addData("target pos", 10473);
                telemetry.update();
                Arm.setTargetPosition(0);
                break;
            default:
                return;
        }
    }

    private void ForeArmControlPreset(int position) {
        ForeArm.setPower(encoderPower);
        switch (position) {
            case 0:
                telemetry.addData("target pos", 0);
                telemetry.update();
                ForeArm.setTargetPosition(0);
                break;
            case 1:
                telemetry.addData("target pos", 884);
                telemetry.update();
                ForeArm.setTargetPosition(-609);
                break;
            case 2:
                telemetry.addData("target pos", 6736);
                telemetry.update();
                ForeArm.setTargetPosition(-422);
                break;
            case 3:
                telemetry.addData("target pos", 10473);
                telemetry.update();
                ForeArm.setTargetPosition(0);
                break;
            default:
                return;
        }
    }

    private void HandControl() {
        Hand.setPosition(Hand.getPosition() == handOpen ? handClosed : handOpen);
    }

    /**
     * Describe this function...
     */
    private void moveXY(double xVal, double yVal, double rVal) {
        omnidirectional(xVal, yVal, rVal);
    }

    /**
     * Describe this function...
     * +speed = forward
     * +turn = clockwise
     * -strafe = right
     */
    private void omnidirectional(double turn, double speed, double strafe) {
//        strafe = strafe * -1;

        double topLeft = speed + turn + strafe;
        double topRight = speed - turn - strafe;
        double bottomLeft = speed + turn - strafe;
        double bottomRight = speed - turn + strafe;

        TopRight.setPower(topRight);
        BottomRight.setPower(bottomRight);
        TopLeft.setPower(topLeft);
        BottomLeft.setPower(bottomLeft);
    }
}