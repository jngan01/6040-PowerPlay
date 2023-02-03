package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class justinJellyMop extends OpMode {
    private DcMotor leftFront;
    private DcMotor rightFront;
    private DcMotor leftBack;
    private DcMotor rightBack;
    private DcMotor topLift;
    private DcMotor bottomLift;
    private DcMotor fourBar;
    private CRServo claw;

    @Override
    // On initialization
    public void init() {

        leftFront = hardwareMap.dcMotor.get("leftFront");
        leftBack = hardwareMap.dcMotor.get("leftBack");
        rightFront = hardwareMap.dcMotor.get("rightFront");
        rightBack = hardwareMap.dcMotor.get("rightBack");
        topLift = hardwareMap.dcMotor.get("liftTop");
        bottomLift = hardwareMap.dcMotor.get("liftBottom");
        fourBar = hardwareMap.dcMotor.get("fourBar");
        claw = hardwareMap.crservo.get("clawIntake");

        //rightFront.setDirection(DcMotorSimple.Direction.REVERSE);
        rightBack.setDirection(DcMotorSimple.Direction.REVERSE);
        rightFront.setDirection(DcMotorSimple.Direction.REVERSE);

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fourBar.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        topLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bottomLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        telemetry.addLine("Initialized"); // Text shown on phone that says "Initialized"
    }

    @Override
    //While program is running
    public void loop() {

// claw
        if(gamepad2.a){
            claw.setPower(.25);
        }else{
            claw.setPower(-.25);
        }

// lift
        topLift.setPower(-gamepad2.left_stick_y );// up
        bottomLift.setPower(gamepad2.left_stick_y);

        topLift.setPower(gamepad2.left_stick_y ); // down
        bottomLift.setPower(-gamepad2.left_stick_y);

//four bar
        if(gamepad2.left_bumper){
            fourBar.setPower(.25);
        }else if(gamepad2.right_bumper){
            fourBar.setPower(-1);
        }

// drive
        leftFront.setPower(-gamepad1.left_stick_y * .75);
        leftBack.setPower(-gamepad1.left_stick_y * .75);
        rightFront.setPower(-gamepad1.left_stick_y * .75);
        rightBack.setPower(-gamepad1.left_stick_y * .75);

        if(gamepad1.right_stick_x < 0){
            leftFront.setPower(-.5);
            rightFront.setPower(.5);
            leftBack.setPower(-.5);
            rightBack.setPower(.5);
        }if(gamepad1.right_stick_x > 0){
            leftFront.setPower(.5);
            rightFront.setPower(-.5);
            leftBack.setPower(.5);
            rightBack.setPower(-.5);
        }

        if(gamepad1.left_bumper){
            leftFront.setPower(-.75);
            leftBack.setPower(.75);
            rightFront.setPower(.75);
            rightBack.setPower(-.75);
        } else if(gamepad1.right_bumper){
            leftFront.setPower(.75);
            leftBack.setPower(-.75);
            rightFront.setPower(-.75);
            rightBack.setPower(.75);
        }
       /* if (Math.abs(gamepad1.left_stick_x) > Math.abs(gamepad1.left_stick_y)) {
            leftFront.setPower(gamepad1.left_stick_x);
            leftBack.setPower(-gamepad1.left_stick_x);
            rightFront.setPower(-gamepad1.left_stick_x);
            rightBack.setPower(gamepad1.left_stick_x);
        } else if (Math.abs(gamepad1.left_stick_y) > Math.abs(gamepad1.left_stick_x)) {
            leftFront.setPower(-gamepad1.left_stick_y);
            leftBack.setPower(-gamepad1.left_stick_y);
            rightFront.setPower(-gamepad1.left_stick_y); //Problem could be here
            rightBack.setPower(-gamepad1.left_stick_y);
        } else {
            if (!gamepad1.right_bumper && !gamepad1.left_bumper && Math.abs(gamepad1.right_stick_x) <= 0.05 && Math.abs(gamepad1.right_stick_y) <= 0.05 && Math.abs(gamepad1.left_stick_x) <= 0.05 && Math.abs(gamepad1.left_stick_y) <= 0.05) {
                leftFront.setPower(0);
                leftBack.setPower(0);
                rightBack.setPower(0);
                rightFront.setPower(0);
            }
        } */
    }
}





