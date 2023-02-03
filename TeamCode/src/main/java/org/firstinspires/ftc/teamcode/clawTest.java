package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class clawTest  extends OpMode{
    private CRServo clawIntake;
    private DcMotor fourBar;
    private DcMotor topLift;
    private DcMotor bottomLift;


    /* double reversePower = -.10;
    double resistance = -fourBar.getPower() + reversePower; */ //Weird stuff don't use
    @Override
    public void init(){
        clawIntake = hardwareMap.crservo.get("clawIntake");
        fourBar = hardwareMap.dcMotor.get("fourBar");
        topLift = hardwareMap.dcMotor.get("liftTop");
        bottomLift = hardwareMap.dcMotor.get("liftBottom");
        fourBar.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        topLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bottomLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    }

    @Override
    public void loop(){
        if(gamepad1.a){
            clawIntake.setPower(.25);
        }else{
            clawIntake.setPower(-.25);
        }

        if(gamepad1.y){
            fourBar.setPower(1);
        }else if(gamepad1.x){
            fourBar.setPower(-1);
        }else if(!gamepad1.y && !gamepad1.x){
            fourBar.setPower(0);
        }

        topLift.setPower(-gamepad1.left_trigger );
        bottomLift.setPower(gamepad1.left_trigger);
        topLift.setPower(gamepad1.right_trigger );
        bottomLift.setPower(-gamepad1.right_trigger);

        }
    }

