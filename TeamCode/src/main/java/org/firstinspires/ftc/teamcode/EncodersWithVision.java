package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.vuforia.HINT;
import com.vuforia.Vuforia;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.ZYX;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;

@Autonomous
public class EncodersWithVision extends LinearOpMode {

    private DcMotor leftFront;
    private DcMotor leftBack;
    private DcMotor rightFront;
    private DcMotor rightBack;

    private int leftFrontPos;
    private int leftBackPos;
    private int rightFrontPos;
    private int rightBackPos;


    private DcMotor fourBar;
    private DcMotor topLift;
    private DcMotor bottomLift;

    private CRServo claw;

    private WebcamName webcam;
    private VuforiaLocalizer vuforia = null;
    private VuforiaTrackables targets = null;

    public static final String JustinRobodogs23 = "AbPAsZf/////AAABmfibwXNw70SrjNg5qBWQi6ohhZf4HQXyJCn4vbC0aLRr+NHm3MyUJXkJxUF2Wk4RqQrqcoJCA3fELgH4SrjbvnsQzMFFb0y/GtXrHfYwzwbVG9Gg3LrOd/Rlet/qI39Q9foADM6Zu9XV21KISqXKamo6DDV8BfOE8vz6z18j7O4hoUfX9JYidlFunwAUFNMvHw5KEreXxAdKO6V2s51kUN1Jus7D9SKsztg7gIlU6D2BC2o7SXu0x8sN2/EqYcNGt9UpeV8SCYXImiIHN3eMzF9U4VKUUzYOzjuU2L+04BByIEtbCZKO2wPwCsK7WSnq65ES1KnO669ZOwt8dudWiE1Pl3dMOkisVmXx23UMrt5J";

    @Override
    public void runOpMode() throws InterruptedException {

        webcam = hardwareMap.get(WebcamName.class, "Webcam 1");

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId); //R.id
        parameters.vuforiaLicenseKey = JustinRobodogs23;
        parameters.cameraName = webcam;
        parameters.useExtendedTracking = false;

        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        int scanned = 0;

        targets = this.vuforia.loadTrackablesFromAsset("JustinFTC23");
        targets.get(0).setName("Sleepy");
        targets.get(2).setName("Catan");
        targets.get(1).setName("Triangle");

        targets.activate();

        leftFront = hardwareMap.dcMotor.get("leftFront");
        leftBack = hardwareMap.dcMotor.get("leftBack");
        rightFront = hardwareMap.dcMotor.get("rightFront");
        rightBack = hardwareMap.dcMotor.get("rightBack");

        fourBar = hardwareMap.dcMotor.get("fourBar");
        topLift = hardwareMap.dcMotor.get("liftTop");
        bottomLift = hardwareMap.dcMotor.get("liftBottom");

        claw = hardwareMap.crservo.get("clawIntake");

        rightFront.setDirection(DcMotorSimple.Direction.REVERSE);
        rightBack.setDirection(DcMotorSimple.Direction.REVERSE);

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        fourBar.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        topLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bottomLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //Encoder Part
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftFrontPos = 0;
        leftBackPos = 0;
        rightFrontPos = 0;
        rightBackPos = 0;


        waitForStart();

        VuforiaTrackableDefaultListener Sleepy = (VuforiaTrackableDefaultListener) targets.get(0).getListener();
        VuforiaTrackableDefaultListener Catan = (VuforiaTrackableDefaultListener) targets.get(2).getListener();
        VuforiaTrackableDefaultListener Triangle = (VuforiaTrackableDefaultListener) targets.get(1).getListener();

        telemetry.addLine("Vuforia Initialized");
        telemetry.update();

        claw.setPower(1);
        sleep(1000);
        fourBar.setPower(.7);
        sleep(800);
        fourBar.setPower(0.25);
        sleep(2000);

        while (opModeIsActive()) {
            if (Triangle.isVisible()) {
                telemetry.addLine("Triforce");
                telemetry.update();
                scanned = 1;

            } else if (Catan.isVisible()) {
                telemetry.addLine("Catan");
                telemetry.update();
                scanned = 2;

            } else if (Sleepy.isVisible()) {
                telemetry.addLine("Sleepy");
                telemetry.update();
                scanned = 3;
            } else {
                telemetry.addLine("nada");
                telemetry.update();
                scanned = 0;
            }
        }



        if (scanned > 0) {
            drive(-450, 450, 450, -450, .25);
            sleep(500);
            drive(820, 820, 820, 820, 0.25);

            fourBar.setPower(-.5);
            sleep(750);
            claw.setPower(-1);
            sleep(750);

            drive(-900, -900, -900, -900, .25);
            drive(250, 250, 250, 250, .25);

            if (scanned == 1) {
                drive(-500, 500, 500, -500, .25);
                drive(1000, 1000, 1000, 1000, .5);
            } else if (scanned == 2) {
                drive(500, -500, -500, 500, .25);
                drive(1000, 1000, 1000, 1000, .5);
            } else if (scanned == 3) {
                drive(1000, -1000, -1000, 1000, .25);
                drive(1000, 1000, 1000, 1000, .5);
            } else{
                drive(-500, 500, 500, -500, .25);
            }
        }

    }


        private void drive ( int lfTarget, int lbTarget, int rfTarget, int rbTarget, double speed){
            leftFrontPos += lfTarget;
            leftBackPos += lbTarget;
            rightFrontPos += rfTarget;
            rightBackPos += rbTarget;

            leftFront.setTargetPosition(leftFrontPos);
            leftBack.setTargetPosition(leftBackPos);
            rightFront.setTargetPosition(rightFrontPos);
            rightBack.setTargetPosition(rightBackPos);

            leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            leftFront.setPower(speed);
            leftBack.setPower(speed);
            rightFront.setPower(speed);
            rightBack.setPower(speed);

            while (opModeIsActive() && leftFront.isBusy() && leftBack.isBusy() && rightFront.isBusy() && rightBack.isBusy()) {
                idle();
            }
        }

    }

