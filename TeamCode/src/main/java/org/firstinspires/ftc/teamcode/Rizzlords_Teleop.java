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
        handOpen = 0.3;
        handClosed = 0;
        Hand.setPosition(handOpen);

        planeLoad = 0.5;
        planeClosed = 0.7;
        PlaneServo.setPosition(planeLoad);

        CurrRotation = 0;

        // initilization blocks, right motor = front right, left motor = front left, arm = back right, hand = back left
        BottomLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        TopLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        waitForStart();
        if (opModeIsActive()) {
            while (opModeIsActive()) {
                WheelControl();
                if(!gamepad1.right_bumper && !gamepad1.left_bumper){
                    ArmControl();
                }
                TuneResetEncoder();
                ForeArmControl();
                HandControl();
                PlaneControl();
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
        double turn = gamepad1.right_stick_x;
        double strafe = -gamepad1.left_stick_x;

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
