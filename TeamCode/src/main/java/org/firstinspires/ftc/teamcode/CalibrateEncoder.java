package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Calibrate Encoder", group = "swagbots")
public class CalibrateEncoder extends LinearOpMode {
    private DcMotor Arm;
    private DcMotor ForeArm;
    private DcMotor HangArm;
    private Servo Hand;
    private Servo PlaneServo;
    private int armEncoder;
    private int handEncoder;
    private int hangEncoder;
    double handPosition;
    double planePosition;

    @Override
    public void runOpMode() throws InterruptedException {
        Arm = hardwareMap.get(DcMotor.class, "Arm1");
        ForeArm = hardwareMap.get(DcMotor.class, "Forearm");
        HangArm = hardwareMap.get(DcMotor.class, "HangArm");
        Hand = hardwareMap.get(Servo.class, "Hand");
        PlaneServo = hardwareMap.get(Servo.class, "PlaneServo");

        handPosition = 0;
        planePosition = 0;

        RunUsingEncoder(Arm, 0.1);
        RunUsingEncoder(ForeArm, 0.1);
//        'yippee'

        RunUsingEncoder(HangArm, 1);

        telemetry.addData("Current Arm Encoder", armEncoder);
        telemetry.addData("Current ForeArm Encoder", handEncoder);
        telemetry.addData("Current HangArm Encoder", hangEncoder);
        telemetry.addData("Current Hand Position", Hand.getPosition());

        waitForStart();
        if(opModeIsActive()){
            while(opModeIsActive()){
                Arm.setTargetPosition(armEncoder);
                ForeArm.setTargetPosition(handEncoder);
                HangArm.setTargetPosition(hangEncoder);
                armEncoder += -gamepad1.right_stick_y;
                handEncoder += -gamepad1.left_stick_y;
                hangEncoder += gamepad1.left_stick_x;
//                while(gamepad1.right_trigger > 0){
//                    hangEncoder += 1;
//                }
//                while(gamepad1.left_trigger > 0){
//                    hangEncoder -= 1;
//                }
                if(gamepad1.dpad_up){
                    handPosition += 0.1;
                    sleep(250);
                } else if(gamepad1.dpad_down) {
                    handPosition += -0.1;
                    sleep(250);
                }
                if(gamepad1.dpad_right){
                    planePosition += 0.1;
                    sleep(250);
                } else if(gamepad1.dpad_left) {
                    planePosition += -0.1;
                    sleep(250);
                }
                Hand.setPosition(handPosition);
                PlaneServo.setPosition(planePosition);
                telemetry.addData("Current Arm Encoder", armEncoder);
                telemetry.addData("Current ForeArm Encoder", handEncoder);
                telemetry.addData("Current HangArm Encoder", hangEncoder);
                telemetry.addData("Current Hand Position", Hand.getPosition());
                telemetry.addData("Current Plane Position", PlaneServo.getPosition());
                telemetry.update();
            }
        }
    }

    private void RunUsingEncoder(DcMotor motor, double power){
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor.setTargetPosition(0);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor.setPower(power);
    }
}
