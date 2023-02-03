package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Autonomous
public class AutoBeanBlue extends LinearOpMode {
    private DcMotor leftFront;
    private DcMotor rightFront;
    private DcMotor leftBack;
    private DcMotor rightBack;
    private CRServo clawIntake;
    private DcMotor fourBar;
    private DcMotor topLift;
    private DcMotor bottomLift;

    @Override
    public void runOpMode() throws InterruptedException {
        leftFront = hardwareMap.dcMotor.get("leftFront");
        rightFront = hardwareMap.dcMotor.get("rightFront");
        leftBack = hardwareMap.dcMotor.get("leftBack");
        rightBack = hardwareMap.dcMotor.get("rightBack");
        clawIntake = hardwareMap.crservo.get("clawIntake");
        fourBar = hardwareMap.dcMotor.get("fourBar");
        topLift = hardwareMap.dcMotor.get("liftTop");
        bottomLift = hardwareMap.dcMotor.get("liftBottom");
        rightBack.setDirection(DcMotorSimple.Direction.REVERSE);


        waitForStart();

        clawIntake.setPower(-25);
        uppyDown(1);
        sleep(1500);
        uppyDown(0);


    }
    public void strafe(double direction) {

        leftFront.setPower(-1 * direction);
        leftBack.setPower(1 * direction);
        rightFront.setPower(1 * direction);
        rightBack.setPower(-1 * direction);

    }
    public void move(double direction) {
        leftFront.setPower(1 * direction);
        leftBack.setPower(1 * direction);
        rightFront.setPower(1 * direction);
        rightBack.setPower(1 * direction);
    }
    public void uppyDown(double uppyDown){
        topLift.setPower(-1 * uppyDown);
        bottomLift.setPower(-1 * uppyDown);
    }
}

