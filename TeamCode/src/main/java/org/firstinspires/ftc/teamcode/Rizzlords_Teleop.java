package org.firstinspires.ftc.teamcode;

// Import needed packages
import android.media.Image;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.RobotLog;
import com.qualcomm.robotcore.util.ElapsedTime;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

// Line 23 defines that this will run in TeleOp mode under the labelled name and group.
// Make sure not to forget the "extends LinearOpMode" part in Line 24
@TeleOp(name = "Rizzlords_Teleop", group = "Rizzlords")
public class Rizzlords_Teleop extends LinearOpMode {

    // Declare all your variables here
    // This includes references to parts on the robot to anything you would like to store
    // You can worry about assigning them later on

    //Drive Motors
    private DcMotor BottomLeft;
    private DcMotor TopRight;
    private DcMotor BottomRight;
    private DcMotor TopLeft;


    //Arm and Hand Motors
    private DcMotor Arm;
    private DcMotor ForeArm;
    private DcMotor HangArm;
    private Servo Hand;
    private Servo PlaneServo;
    double handOpen, handClosed;
    double planeLoad, planeClosed;

    private double CurrRotation;

    private ElapsedTime runtime = new ElapsedTime();
    private double encoderPower;
    private int armEncoder;
    private int foreArmEncoder;


    //AUTO VARS
    private int stageGlobal;

    double refAngAdj, refAngOriginal, refAng;


    IMU imu;

    // The function below is where most of your coding will be at
    @Override
    public void runOpMode() throws InterruptedException {

        // Assign the variables declared recently
        // Variables that reference to hardware components on your robot are assigned with the function hardwareMap.get()
        // The first argument is the type of hardware and the second is the name that you have assigned to that piece of hardware
        // on the phone app.

        //Drive motors
        BottomLeft = hardwareMap.get(DcMotor.class, "Bottom Left");
        TopRight = hardwareMap.get(DcMotor.class, "Top Right");
        BottomRight = hardwareMap.get(DcMotor.class, "Bottom Right");
        TopLeft = hardwareMap.get(DcMotor.class, "Top Left");

        //Arm hand motors
        Arm = hardwareMap.get(DcMotor.class, "Arm1");
        ForeArm = hardwareMap.get(DcMotor.class, "Forearm");
        Hand = hardwareMap.get(Servo.class, "Hand");
        HangArm = hardwareMap.get(DcMotor.class, "HangArm");
        PlaneServo = hardwareMap.get(Servo.class, "PlaneServo");
        encoderPower = .3;
        RunUsingEncoder(Arm);
        RunUsingEncoder(ForeArm);
        RunUsingEncoder(HangArm);
        handOpen = 1;//TODO
        handClosed = 0.7;
        Hand.setPosition(handOpen);

        planeLoad = 0.5;
        planeClosed = 0.7;
        PlaneServo.setPosition(planeLoad);


        //AUTO VARS
        stageGlobal = -1;

        // The below code is for some initialization for the "imu" that is built into the control hub.
        // This is pretty cool because it can measure the orientation of the robot relative to it's original rotation.
        // Feel free to google more about the imu if you ever need to use it for solving a particular problem.
        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters myIMUparameters;

        myIMUparameters = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.FORWARD,
                        RevHubOrientationOnRobot.UsbFacingDirection.UP
                )
        );
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.UP;
        RevHubOrientationOnRobot.UsbFacingDirection  usbDirection  = RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;

        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);

        // Now initialize the IMU with this mounting orientation
        // Note: if you choose two conflicting directions, this initialization will cause a code exception.
        imu.initialize(new IMU.Parameters(orientationOnRobot));

        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        refAngAdj = Math.PI/2;
        refAngOriginal = orientation.getYaw(AngleUnit.RADIANS);
        refAng = refAngOriginal + refAngAdj;

        CurrRotation = 0;

        // If any of the motors do not spin in a desired direction by default, use the "setDirection" method to reverse it
        // This will be particularly useful for configuring your wheels to work with omnidirectional movement
        BottomLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        TopRight.setDirection(DcMotorSimple.Direction.REVERSE);



//        TopLeft.setDirection(DcMotorSimple.Direction.REVERSE);
//        BottomRight.setDirection(DcMotorSimple.Direction.REVERSE);

        // All of the code under the "if" and "while" statement is what is run over and over by the robot
        waitForStart();
        if (opModeIsActive()) {
            while (opModeIsActive()) {
                // What I did here was call a bunch of functions that I declared later on to perform specific tasks.
                // This may help a bit with organization.

                //MANUAL
//                WheelControl();
                AbsoluteDrive();
//                MecanumInput();
//                MecanumInputRelative();

                // Use "gamepad1.{some button from the list Android Studio pops up with}"
                // to check if a button on the controller is pressed
                if(!gamepad1.right_bumper && !gamepad1.left_bumper){
                    ArmControl();
                }
                TuneResetEncoder();
                ForeArmControl();
                HandControl();
                PlaneControl();
                HangControl();
                
                // The below function logs any information to the phone during runtime
                // that will appear in a black box at the bottom of the app.
                telemetry.addData("Hand Position", Hand.getPosition());

                // Retrieve Rotational Angles and Velocities

                orientation = imu.getRobotYawPitchRollAngles();
                AngularVelocity angularVelocity = imu.getRobotAngularVelocity(AngleUnit.DEGREES);

                if(gamepad1.x){
                    refAngAdj *= -1;
                    refAng = refAngOriginal + refAngAdj;
                }

                // TODO: console log feedback for which directions the refAng is
                if(refAngAdj > 0){
                    telemetry.addData("<", "----");
                } else{
                    telemetry.addData("----", ">");
                }

                telemetry.addData("Yaw (Z)", "%.2f Deg. (Heading)", orientation.getYaw(AngleUnit.DEGREES));
                telemetry.addData("Pitch (X)", "%.2f Deg.", orientation.getPitch(AngleUnit.DEGREES));
                telemetry.addData("Roll (Y)", "%.2f Deg.\n", orientation.getRoll(AngleUnit.DEGREES));
//                telemetry.addData("Yaw (Z) velocity", "%.2f Deg/Sec", angularVelocity.zRotationRate);
//                telemetry.addData("Pitch (X) velocity", "%.2f Deg/Sec", angularVelocity.xRotationRate);
//                telemetry.addData("Roll (Y) velocity", "%.2f Deg/Sec", angularVelocity.yRotationRate);

                telemetry.update();
            }
        }
    }

    // If you need to control a motor not by power but rather position, you need indicate it to run using encoder
    private void RunUsingEncoder(DcMotor motor){
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor.setTargetPosition(0);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor.setPower(.1);
    }



    //AUTO
    // You can skim over this section if you'd like until I define a "MANUAL" section.
    // Basically everything here is code of trying to simplify the controls for the player which didn't end up working out.

    /**
     * Describe this function...
     * +speed = forward
     * +turn = clockwise
     * -strafe = right
     */
//    private void Ape(int stage){
//        //the robot will always be facing the opposite side except when placing pixels
//        switch(stage){
//            case -1:
//                //move back slowly to starting tile
//                //default pos
//                //rotate 90 deg to face opposite side
//                Travel(-0.3, 0, 0, 0.2);
//                break;
//            case 0:
//                //move to front stage
//                Travel(0, 0.3, 0, 5);
//                break;
//            case 1:
//                //move to pixels
//                Travel(0, 0, 0.3, 1);
//                break;
//            case 2:
//                //pickup pos
//                //handcontrol
//                //default pos
//                break;
//            case 3:
//                //move to front stage starting tile
//                break;
//            case 4:
//                //move to tile in front of board (at a moderate speed)
//                break;
//            case 5:
//                //rotate 90 deg to face board
//                //placing pos
//                //hand control
//                stageGlobal = -1;
//                break;
//            default:
//                return;
//        }
//    }

    // This function served to set both the arm and forearm to a desired position by calling two other functions that
    // set the specific arm position based on the input preset (defined by an integer).
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

    // Controls robot hanging mechanism
    private void HangControl(){
        int raised = 0;
        int retracted = 0;
        boolean isRaised = false;
        if(gamepad1.dpad_right){
            HangArm.setTargetPosition(isRaised ? retracted : raised);
            isRaised = isRaised ? false : true;
//            telemetry.addData("current hand position: ", Hand.getPosition());
            sleep(250);
        }
    }

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



    private void HandControlAuto() {
        Hand.setPosition(Hand.getPosition() == handOpen ? handClosed : handOpen);
    }

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



    // MANUAL   !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

    // An issue we encountered was that the arm motor would "forget" it's initial position.
    // It was a bit annoying having to adjust the motor everytime using the "ResetEncoder" class so
    // this function was just to do it while running the teleop code.
    private void TuneResetEncoder(){
        if(gamepad1.right_bumper){
            armEncoder += 1;
        } else if(gamepad1.left_bumper){
            armEncoder -= 1;
        }
        if(gamepad1.y){
            Arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            armEncoder = 0;
            RunUsingEncoder(Arm);
        }
        Arm.setTargetPosition(armEncoder);
    }


    // The functions "ArmControl" and "ForeArmControl" serve to set the arm and forearm to a desired position
    // based on which dpad direction was pressed.

    // arm/forearm control notes
    // top - default
    // ready to pick up - left
    // pick up pos - down
    // place onto board pos - right
    private void ArmControl(){
        int MAX = 933;
        int MIN = -30;
        if(gamepad1.dpad_up){
            armEncoder = 0;
        }
        else if(gamepad1.dpad_left){
            armEncoder = -67;
        }
        else if(gamepad1.dpad_down){
            armEncoder = 161;
        }

        telemetry.addData("Arm Encoder:", armEncoder);
        Arm.setPower(encoderPower);
        Arm.setTargetPosition(armEncoder);
//        if (armEncoder < MIN) {
//            armEncoder = MIN;
//        }
//        else if (armEncoder > MAX) {
//            armEncoder = MAX;
//        }
    }

    private void ForeArmControl(){
        int MAX = 0;
        int MIN = -800;
        if(gamepad1.dpad_up){
            foreArmEncoder = 0;
        }
        else if(gamepad1.dpad_left){
            foreArmEncoder = -609;
        }
        else if(gamepad1.dpad_down){
            foreArmEncoder = -422;
        }

        telemetry.addData("Forearm Encoder:", foreArmEncoder);
        ForeArm.setPower(encoderPower);
        ForeArm.setTargetPosition(foreArmEncoder);
//        if (foreArmEncoder < MIN) {
//            foreArmEncoder = MIN;
//        }
//        else if (foreArmEncoder > MAX) {
//            foreArmEncoder = MAX;
//        }
    }

    // This function allowed the player to toggle the hand between open and closed when they would
    // press the "A" button on the controller.
    private void HandControl(){
        if(gamepad1.a){
            Hand.setPosition(Hand.getPosition() == handOpen ? handClosed : handOpen);
            telemetry.addData("current hand position: ", Hand.getPosition());
            sleep(250);
        }
    }

    // This function served to launch our paper plane by opening a servo that would let loose an elastic band
    // after the player presses the button "B"
    private void PlaneControl(){
        if(gamepad1.b){
            PlaneServo.setPosition(PlaneServo.getPosition() == planeLoad ? planeClosed : planeLoad);
            telemetry.addData("current plane position: ", PlaneServo.getPosition());
            sleep(250);
        }
    }

    // This was an attempt to use the IMU built into the control hub to drive the robot with directionality relative
    // to the player. I remember it worked for a few matches but broke down afterwards for some reason I never figured out.
    private void AbsoluteDrive(){
        double vertical;
        double horizontal;
        double pivot;
        TopRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BottomRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        TopLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BottomLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        double speedDiv;
        if(gamepad1.left_trigger > 0){
            speedDiv = 1;
        } else if(gamepad1.right_trigger > 0){
            speedDiv = 0.3;
        } else {
            speedDiv = 0.5;
        }

        double speedI = gamepad1.left_stick_y;
        double turnI = gamepad1.left_stick_x;
        double strafe = -gamepad1.right_stick_x;

        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        double currRot = orientation.getYaw(AngleUnit.RADIANS);
        double theta = currRot - refAng;
        double radius = Math.sqrt(Math.pow(speedI, 2) + Math.pow(turnI, 2));

        double speed = Math.sin(theta + Math.atan2(speedI, turnI)) * radius;
        double turn = Math.cos(theta + Math.atan2(speedI, turnI)) * radius;

        if(speed > 1){
            speed = speedI * Math.cos(Math.PI/2 - theta) - turnI * Math.sin(theta);
        }

        if(turn > 1){
            turn = turnI * Math.cos(theta) - speed * Math.sin(Math.PI/2 - theta);
        }

        telemetry.addData("speed", speed);
        telemetry.addData("strafe", turn);

        double topLeft = speed + turn + strafe;
        double topRight = speed - turn - strafe;
        double bottomLeft = speed + turn - strafe;
        double bottomRight = speed - turn + strafe;

        TopRight.setPower(speedDiv * topRight);
        BottomRight.setPower(speedDiv * bottomRight);
        TopLeft.setPower(speedDiv * topLeft);
        BottomLeft.setPower(speedDiv * bottomLeft);

//        telemetry.addData("BottomLeft", bottomLeft);
//        telemetry.addData("TopRight", topRight);
//        telemetry.addData("TopLeft", topLeft);
//        telemetry.addData("bottomRight", bottomRight);
    }


    // This was the main function I used for omnidirectional drive. Note that this version makes it so that if the
    // player tells the robot to go forward, it goes forward relative to the robot and not the player.
    // I'll try my best to provide solid comments in this function, but I suggest checking out the following 2
    // youtube videos because they explain it very well.
    // https://youtu.be/gnSW2QpkGXQ?si=vkYwZCwndqOuL_L2   https://youtu.be/cXrDz1cb8N0?si=3_HsP2rzObhypDF6
    private void WheelControl() {
        double vertical;
        double horizontal;
        double pivot;

        // Ensure the motors are not using encoder.
        TopRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BottomRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        TopLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BottomLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // The variable "speedDiv" served to control the power of the drive, where if the player held down
        // left trigger, the robot would go faster, and vice versa with right trigger
        double speedDiv;
        if(gamepad1.left_trigger > 0){
            speedDiv = 1;
        } else if(gamepad1.right_trigger > 0){
            speedDiv = 0.3;
        } else {
            speedDiv = 0.5;
        }

        // Declare speed "forward/backward", turn "clockwise/anticlockwise", and strafe "left/right"
        double speed = gamepad1.left_stick_y;
        double turn = gamepad1.left_stick_x;
        double strafe = -gamepad1.right_stick_x;

        // The code below does all the magic regarding omnidirectional drive. Note that I changed up the
        // configuration below because the original one found in the videos didn't work, although I would suggest
        // first testing out what is instructed by the videos and then playing around if there are issues.
        double topLeft = speed + turn + strafe;
        double topRight = speed - turn - strafe;
        double bottomLeft = speed + turn - strafe;
        double bottomRight = speed - turn + strafe;

        // Set the wheels' powers based off of the variables.
        TopRight.setPower(speedDiv * topRight);
        BottomRight.setPower(speedDiv * bottomRight);
        TopLeft.setPower(speedDiv * topLeft);
        BottomLeft.setPower(speedDiv * bottomLeft);

        telemetry.addData("BottomLeft", bottomLeft);
        telemetry.addData("TopRight", topRight);
        telemetry.addData("TopLeft", topLeft);
        telemetry.addData("bottomRight", bottomRight);
    }

    // If I remember correctly all the remaining functions were to use an alternative method of
    // omnidirectional driving that I didn't stick to.
    // Learn more about it in this timestamped video: https://youtu.be/gnSW2QpkGXQ?si=QPBge7rfSobD5yH7&t=78
    private void MecanumInput(){
        double y = gamepad1.left_stick_y;
        double turn = gamepad1.left_stick_x;
        double x = -gamepad1.right_stick_x;

//        double x = gamepad1.left_stick_x;
//        double y = -gamepad1.left_stick_y;
//        double turn = gamepad1.right_stick_x;

        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        double currRot = orientation.getYaw(AngleUnit.RADIANS);
        double deltaFromRef = currRot - refAng;
        telemetry.addData("deltaFromRef:", deltaFromRef);

        double input = Math.atan2(y, x) - Math.PI;
        double theta = -deltaFromRef + input;

        telemetry.addData("theta:", theta);

        double power = Math.hypot(x, y);

        MecanumAng(theta, power, turn);
    }

    private void MecanumInputRelative(){
        double x = -gamepad1.left_stick_x;
        double y = -gamepad1.left_stick_y;
        double turn = gamepad1.right_stick_x;

//        double y = gamepad1.left_stick_y;
//        double turn = gamepad1.left_stick_x;
//        double x = -gamepad1.right_stick_x;

        telemetry.addData("x:", x);
        telemetry.addData("y:", y);

        double theta = Math.atan2(y, x);
        double power = Math.hypot(x, y);

        telemetry.addData("theta:", theta);

        MecanumAng(theta, power, turn);
    }

    private void MecanumAng(double theta, double power, double turn){

        TopRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BottomRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        TopLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BottomLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        double speedDiv;
        if(gamepad1.left_trigger > 0){
            speedDiv = 1;
        } else if(gamepad1.right_trigger > 0){
            speedDiv = 0.3;
        } else {
            speedDiv = 0.5;
        }

        double sin = Math.sin(theta - Math.PI/4);
        double cos = Math.cos(theta - Math.PI/4);
        double max = Math.max(Math.abs(sin), Math.abs(cos));

        double leftFront = power * cos/max + turn;
        double rightFront = power * sin/max - turn;
        double leftRear = power * sin/max + turn;
        double rightRear = power * cos/max - turn;

        //TODO: speedDiv control

        if((power + Math.abs(turn)) > 1){
            leftFront /= power + turn;
            rightFront /= power + turn;
            leftRear /= power + turn;
            rightRear /= power + turn;
        }

        TopRight.setPower(speedDiv * rightFront);
        BottomRight.setPower(speedDiv * rightRear);
        TopLeft.setPower(speedDiv * leftFront);
        BottomLeft.setPower(speedDiv * leftRear);
    }
}
