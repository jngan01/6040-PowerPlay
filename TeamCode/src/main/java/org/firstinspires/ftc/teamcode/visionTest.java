package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.ZYX;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;


@Autonomous(name = "visionTest")
public class visionTest extends LinearOpMode{
    VuforiaLocalizer vuforiaLocalizer;
    VuforiaLocalizer.Parameters parameters;
    VuforiaTrackable visionTarget;
    VuforiaTrackableDefaultListener listener;

    private WebcamName webcam;

    private OpenGLMatrix lastLocation = null;
    private VuforiaLocalizer vuforia = null;
    private VuforiaTrackables targets = null;

    public static final String JustinRobodogs23 = "AbPAsZf/////AAABmfibwXNw70SrjNg5qBWQi6ohhZf4HQXyJCn4vbC0aLRr+NHm3MyUJXkJxUF2Wk4RqQrqcoJCA3fELgH4SrjbvnsQzMFFb0y/GtXrHfYwzwbVG9Gg3LrOd/Rlet/qI39Q9foADM6Zu9XV21KISqXKamo6DDV8BfOE8vz6z18j7O4hoUfX9JYidlFunwAUFNMvHw5KEreXxAdKO6V2s51kUN1Jus7D9SKsztg7gIlU6D2BC2o7SXu0x8sN2/EqYcNGt9UpeV8SCYXImiIHN3eMzF9U4VKUUzYOzjuU2L+04BByIEtbCZKO2wPwCsK7WSnq65ES1KnO669ZOwt8dudWiE1Pl3dMOkisVmXx23UMrt5J"; //License Key

    @Override
    public void runOpMode() throws InterruptedException{
        //vuforia setup
        webcam = hardwareMap.get(WebcamName.class, "Webcam 1");

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        parameters = new VuforiaLocalizer.Parameters(R.id.cameraMonitorViewId);
        parameters.vuforiaLicenseKey = JustinRobodogs23;
        parameters.cameraName = webcam;
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;

        vuforia = ClassFactory.getInstance().createVuforia(parameters);



        targets = vuforia.loadTrackablesFromAsset("JustinFTC23");
        targets.get(0).setName("Yellow1");
        targets.get(1).setName("Green2");
        targets.get(2).setName("Purple3");

        targets.activate();

        waitForStart();

        VuforiaTrackableDefaultListener yellow1 = (VuforiaTrackableDefaultListener) targets.get(0).getListener();
        VuforiaTrackableDefaultListener green2 = (VuforiaTrackableDefaultListener) targets.get(1).getListener();
        VuforiaTrackableDefaultListener purple3 = (VuforiaTrackableDefaultListener) targets.get(2).getListener();

        telemetry.addLine("Vuforia Initialized");
        telemetry.update();

        if(yellow1.isVisible()){
            telemetry.addLine("Side 1");
            telemetry.update();
        }else if(green2.isVisible()){
            telemetry.addLine("Side 2");
            telemetry.update();
        }else if (purple3.isVisible()){
            telemetry.addLine("Side 3");
            telemetry.update();
        }else {
            telemetry.addLine("nada");
            telemetry.update();
        }

        while(opModeIsActive()) {

            telemetry.update();
            idle();
        }

    }

}
