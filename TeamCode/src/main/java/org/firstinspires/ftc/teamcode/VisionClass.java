package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.arcrobotics.ftclib.geometry.Vector2d;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Rect;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;

import java.util.HashMap;
import java.util.List;

public class VisionClass {
    private final Telemetry telemetry;
    OpenCvWebcam camera;
    public final static int camera_width = 1280;
    public final static int camera_height = 720;

    public VisionClass(VisionOpMode opMode, Telemetry telemetry, HardwareMap hardwareMap){
        this.telemetry = telemetry;
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"));
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(camera_width, camera_height, OpenCvCameraRotation.UPRIGHT);
                camera.setPipeline(new PipeLine());
                FtcDashboard.getInstance().startCameraStream(camera,0);
            }
            @Override
            public void onError(int errorCode) {}
        });
    }
    private class PipeLine extends OpenCvPipeline {

        @Override
        public Mat processFrame(Mat input) {
            Mat processed = ShapeDetectionUtil.processImage(input);
            ShapeDetectionUtil.markOuterContour(processed,input);

            List<MatOfPoint> filteredContours = ShapeDetectionUtil.getAllFilteredContours(processed);
            MatOfPoint[] filteredContoursArr = filteredContours.toArray(new MatOfPoint[0]);
            int largestContour =0;
            for(int i=0;i<filteredContoursArr.length;i++){
                if(Imgproc.contourArea(filteredContoursArr[largestContour])<=Imgproc.contourArea(filteredContoursArr[i]))largestContour=i;
            }
            Rect rect= Imgproc.boundingRect(filteredContoursArr[largestContour]);
            int x = (rect.x+ rect.width + rect.x)/2;//the center of the rect
            int y = (rect.y+ rect.height + rect.y)/2;//the center of the rect
            VisionOpMode.objectPos=new Vector2d(x,y);
            for (MatOfPoint contour: filteredContours) {
                contour.release();
            }
            processed.release();
            return input;
        }
    }
    public void stopCamera(){
        camera.closeCameraDevice();
    }

}

//public class Example {
//    public static void main(String args[]){
//        MyThread t1 = new MyThread();
//        t1.start();
//
//        MyThread t2 = new MyThread();
//        t2.start();
//    }
//}
