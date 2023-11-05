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
    private Servo Hand;
    private int armEncoder;
    private int handEncoder;
    double handPosition;

    @Override
    public void runOpMode() throws InterruptedException {
        Arm = hardwareMap.get(DcMotor.class, "Arm1");
        ForeArm = hardwareMap.get(DcMotor.class, "Forearm");
        Hand = hardwareMap.get(Servo.class, "Hand");

        handPosition = 0;

        RunUsingEncoder(Arm);
        RunUsingEncoder(ForeArm);

        telemetry.addData("Current Arm Encoder", armEncoder);
        telemetry.addData("Current ForeArm Encoder", handEncoder);
        telemetry.addData("Current Hand Position", Hand.getPosition());

        waitForStart();
        if(opModeIsActive()){
            while(opModeIsActive()){
                Arm.setTargetPosition(armEncoder);
                ForeArm.setTargetPosition(handEncoder);
                armEncoder += -gamepad1.right_stick_y;
                handEncoder += -gamepad1.left_stick_y;
                if(gamepad1.dpad_up){
                    handPosition += 0.1;
                    sleep(250);
                } else if(gamepad1.dpad_down) {
                    handPosition += -0.1;
                    sleep(250);
                }
                Hand.setPosition(handPosition);
                telemetry.addData("Current Arm Encoder", armEncoder);
                telemetry.addData("Current ForeArm Encoder", handEncoder);
                telemetry.addData("Current Hand Position", Hand.getPosition());
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
}
