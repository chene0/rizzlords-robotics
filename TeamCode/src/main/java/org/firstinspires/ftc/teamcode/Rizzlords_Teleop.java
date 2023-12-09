package org.firstinspires.ftc.teamcode;

import android.media.Image;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.RobotLog;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "Rizzlords_Teleop", group = "Rizzlords")
public class Rizzlords_Teleop extends LinearOpMode {

    //Drive Motors
    private DcMotor BottomLeft;
    private DcMotor TopRight;
    private DcMotor BottomRight;
    private DcMotor TopLeft;


    //Arm and Hand Motors
    private DcMotor Arm;
    private DcMotor ForeArm;
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


    @Override
    public void runOpMode() throws InterruptedException {

        //Drive motors
        BottomLeft = hardwareMap.get(DcMotor.class, "Bottom Left");
        TopRight = hardwareMap.get(DcMotor.class, "Top Right");
        BottomRight = hardwareMap.get(DcMotor.class, "Bottom Right");
        TopLeft = hardwareMap.get(DcMotor.class, "Top Left");

        //Arm hand motors
        Arm = hardwareMap.get(DcMotor.class, "Arm1");
        ForeArm = hardwareMap.get(DcMotor.class, "Forearm");
        Hand = hardwareMap.get(Servo.class, "Hand");
        PlaneServo = hardwareMap.get(Servo.class, "PlaneServo");
        encoderPower = .3;
        RunUsingEncoder(Arm);
        RunUsingEncoder(ForeArm);
        handOpen = 1;//TODO
        handClosed = 0.7;
        Hand.setPosition(handOpen);

        planeLoad = 0.5;
        planeClosed = 0.7;
        PlaneServo.setPosition(planeLoad);


        //AUTO VARS
        stageGlobal = -1;


        CurrRotation = 0;

        // initilization blocks, right motor = front right, left motor = front left, arm = back right, hand = back left
        BottomLeft.setDirection(DcMotorSimple.Direction.REVERSE);
//        TopLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        TopRight.setDirection(DcMotorSimple.Direction.REVERSE);
        waitForStart();
        if (opModeIsActive()) {
            while (opModeIsActive()) {
                //MANUAL
                WheelControl();
                if(!gamepad1.right_bumper && !gamepad1.left_bumper){
                    ArmControl();
                }
                TuneResetEncoder();
                ForeArmControl();
                HandControl();
                HandRecalibrate();
                PlaneControl();
                telemetry.addData("Hand Position", Hand.getPosition());

                telemetry.update();
            }
        }
    }

    private void RunUsingEncoder(DcMotor motor){
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor.setTargetPosition(0);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor.setPower(.1);
    }



    //AUTO

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



    //MANUAL
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
        else if(gamepad1.dpad_right){
//            armEncoder = 900;
//            TEMPORARILY DISABLED
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
        else if(gamepad1.dpad_right){
//            foreArmEncoder = -713;
//            TEMPORARILY DISABLED
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

    private void HandControl(){
        if(gamepad1.a){
            Hand.setPosition(Hand.getPosition() == handOpen ? handClosed : handOpen);
            telemetry.addData("current hand position: ", Hand.getPosition());
            sleep(250);
        }
    }

    private void HandRecalibrate(){
        if(gamepad1.x){
            // handOpen = 0
            int state = (Hand.getPosition() == handOpen) ? 0 : 1;
            handOpen -= 0.0025;
            handClosed -= 0.0025;
            Hand.setPosition(state == 0 ? handOpen : handClosed);
            sleep(250);
        }
    }

    private void PlaneControl(){
        if(gamepad1.b){
            PlaneServo.setPosition(PlaneServo.getPosition() == planeLoad ? planeClosed : planeLoad);
            telemetry.addData("current plane position: ", PlaneServo.getPosition());
            sleep(250);
        }
    }

    private void WheelControl() {
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

        double speed = gamepad1.left_stick_y;
        double turn = gamepad1.left_stick_x;
        double strafe = -gamepad1.right_stick_x;

        double topLeft = speed + turn + strafe;
        double topRight = speed - turn - strafe;
        double bottomLeft = speed + turn - strafe;
        double bottomRight = speed - turn + strafe;

        TopRight.setPower(speedDiv * topRight);
        BottomRight.setPower(speedDiv * bottomRight);
        TopLeft.setPower(speedDiv * topLeft);
        BottomLeft.setPower(speedDiv * bottomLeft);

        telemetry.addData("BottomLeft", bottomLeft);
        telemetry.addData("TopRight", topRight);
        telemetry.addData("TopLeft", topLeft);
        telemetry.addData("bottomRight", bottomRight);
    }
}
