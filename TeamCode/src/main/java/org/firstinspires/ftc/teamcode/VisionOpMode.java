package org.firstinspires.ftc.teamcode;

import android.graphics.Bitmap;

import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Vector2d;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.hardware.motors.CRServo;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.opencv.core.Mat;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class VisionOpMode extends LinearOpMode {
    //PID values for tuning
    public static double yP=1;
    public static double yI=0;
    public static double yD=0;
    public static double xP=1;
    public static double xI=0;
    public static double xD=0;
    private ElapsedTime runtime = new ElapsedTime();
    CRServo xServo;
    CRServo yServo;
    public static Vector2d objectPos;
    VisionClass visionClass;

    @Override
    public void runOpMode() throws InterruptedException {

        //init
        xServo = new CRServo(hardwareMap,"x servo");
        yServo = new CRServo(hardwareMap,"y servo");
        PIDController servoXPID = new PIDController(xP,xI,xD);
        PIDController servoYPID = new PIDController(yP,yI,yD);
        servoYPID.setSetPoint(0);
        servoXPID.setSetPoint(0);
        objectPos = new Vector2d(0,0);
        visionClass = new VisionClass(this,telemetry,hardwareMap);

        waitForStart();

        runtime.reset();
        //code
        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            xServo.set(servoXPID.calculate(objectPos.getX()));
            yServo.set(servoYPID.calculate(objectPos.getY()));
        }
        visionClass.stopCamera();
    }
}
