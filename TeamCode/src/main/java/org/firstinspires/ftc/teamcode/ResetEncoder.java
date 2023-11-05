package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "Reset Encoder", group = "swagbots")
public class ResetEncoder extends LinearOpMode {

    private DcMotor Arm;
    private DcMotor ForeArm;
    private int armEncoder;
    private int handEncoder;

    @Override
    public void runOpMode() throws InterruptedException {
        Arm = hardwareMap.get(DcMotor.class, "Arm1");
        ForeArm = hardwareMap.get(DcMotor.class, "Forearm");

        RunUsingEncoder(Arm);
        RunUsingEncoder(ForeArm);

        waitForStart();
        if(opModeIsActive()){
            while(opModeIsActive()){
                if(gamepad1.a){
                    Arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    ForeArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                }
                Arm.setTargetPosition(armEncoder);
                ForeArm.setTargetPosition(handEncoder);
                armEncoder += -gamepad1.right_stick_y;
                handEncoder += -gamepad1.left_stick_y;
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
