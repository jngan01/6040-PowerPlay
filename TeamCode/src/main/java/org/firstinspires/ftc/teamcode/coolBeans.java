package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp
public class coolBeans extends OpMode
{
    private DcMotor leftFront; // Variable names of DC motors
    private DcMotor rightFront;
    private DcMotor leftBack;
    private DcMotor rightBack;
    private DcMotor uppyAndDown;
    private CRServo spinMeRightRound;  // Servo variable name
    private CRServo claw;

    @Override
    // On initialization
    public void init() {
        // Text in parenthesis are the names listed in the android phone. This assigns variable names to android names.
        leftFront = hardwareMap.dcMotor.get("leftFront");
        rightFront = hardwareMap.dcMotor.get("rightFront");
        leftBack = hardwareMap.dcMotor.get("leftBack");
        rightBack = hardwareMap.dcMotor.get("rightBack");
        uppyAndDown = hardwareMap.dcMotor.get("lift");
        spinMeRightRound = hardwareMap.crservo.get("duckGoSpin");
        claw = hardwareMap.crservo.get("clawIntake");

        telemetry.addLine("Initialized"); // Text shown on phone that says "Initialized"
    }

    @Override
    //While program is running
    public void loop() {
        leftFront.setPower(-gamepad1.left_stick_y);
        leftBack.setPower(-gamepad1.left_stick_y);
        rightFront.setPower(-gamepad1.right_stick_y);
        rightBack.setPower(-gamepad1.right_stick_y);
        uppyAndDown.setPower(gamepad1.left_trigger);
        uppyAndDown.setPower(-gamepad1.right_trigger);

        if(gamepad1.left_bumper) {
            claw.setPower(1);
        }
        if(gamepad1.right_bumper){
            claw.setPower(-1);
        }

        if(gamepad1.a) {
            spinMeRightRound.setPower(-1);
        }
        if(gamepad1.b){
            spinMeRightRound.setPower(1);
        }
        if(gamepad1.y) {
            spinMeRightRound.setPower(0);
        }


    }

}

