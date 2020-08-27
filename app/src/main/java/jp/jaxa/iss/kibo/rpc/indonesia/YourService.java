package jp.jaxa.iss.kibo.rpc.indonesia;

import gov.nasa.arc.astrobee.Kinematics;
import jp.jaxa.iss.kibo.rpc.api.KiboRpcService;
import jp.jaxa.iss.kibo.rpc.indonesia.imageFileUtils;

import android.graphics.Bitmap;
import android.os.Environment;
import android.util.Log;

import gov.nasa.arc.astrobee.Result;
import gov.nasa.arc.astrobee.types.Point;
import gov.nasa.arc.astrobee.types.Quaternion;

import com.google.zxing.BinaryBitmap;
import com.google.zxing.DecodeHintType;
import com.google.zxing.PlanarYUVLuminanceSource;
import com.google.zxing.RGBLuminanceSource;
import com.google.zxing.common.HybridBinarizer;
import com.google.zxing.qrcode.QRCodeReader;

import org.apache.commons.io.FileUtils;
import org.opencv.android.Utils;
import org.opencv.aruco.Aruco;
import org.opencv.aruco.DetectorParameters;
import org.opencv.aruco.Dictionary;
import org.opencv.core.CvException;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;

import static org.opencv.core.Core.BORDER_CONSTANT;
import static org.opencv.core.Core.addWeighted;
import static org.opencv.core.CvType.CV_16SC2;
import static org.opencv.imgcodecs.Imgcodecs.imread;
import static org.opencv.imgproc.Imgproc.GaussianBlur;
import static org.opencv.imgproc.Imgproc.INTER_LINEAR;
import static org.opencv.imgproc.Imgproc.filter2D;
import static org.opencv.imgproc.Imgproc.initUndistortRectifyMap;
import static org.opencv.imgproc.Imgproc.remap;

import java.io.File;
import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.Map;

/**
 * Class meant to handle commands from the Ground Data System and execute them in Astrobee
 */

//Struct like container for QR position and quaternion string data
class QRData {

    private static String posX;
    private static String posY;
    private static String posZ;
    private static String quaX;
    private static String quaY;
    private static String quaZ;

    private static Double radX;
    private static Double radY;
    private static Double radZ;
    static String Ar_Id;

    static void storePosition(String recData){
        String[] arrOfStr;

        //split the data to acquire only the component's value
        arrOfStr = recData.split(",", 6);

        //remove all white spaces
        posX = arrOfStr[1].replaceAll("\\s+",""); //x
        posY = arrOfStr[3].replaceAll("\\s+",""); //y
        posZ = arrOfStr[5].replaceAll("\\s+",""); //z

    }

    static void storeQuaternion(String recData){
        String[] arrOfStr;

        //split the data to acquire only the component's value
        arrOfStr = recData.split(",", 6);

        //remove all white spaces
        quaX = arrOfStr[1].replaceAll("\\s+",""); //x
        quaY = arrOfStr[3].replaceAll("\\s+",""); //y
        quaZ = arrOfStr[5].replaceAll("\\s+",""); //z


    }

    static double getPosX(){
        return Float.parseFloat(posX);
    }

    static double getPosY(){
        return Float.parseFloat(posY);
    }

    static double getPosZ(){
        return Float.parseFloat(posZ);
    }

    static double getQuaX(){
        return Float.parseFloat(quaX);
    }

    static double getQuaY(){
        return Float.parseFloat(quaY);
    }

    static double getQuaZ(){
        return Float.parseFloat(quaZ);
    }

    static double getRadX(){ return radX; }

    static double getRadY(){ return radX; }

    static double getRadZ(){ return radX; }

    static double getQuaW(){

        if(QuaternionIsAvailable()){

            return 1.0 - Math.pow(Double.parseDouble(quaX), 2.0)
                    - Math.pow(Double.parseDouble(quaY), 2.0)
                    - Math.pow(Double.parseDouble(quaZ), 2.0);

        } else {

            return -1.0;
        }

    }

    static Boolean PositionIsAvailable(){
        return posX != null && posY != null && posZ != null;
    }

    static Boolean QuaternionIsAvailable(){
        return quaX != null && quaY != null && quaZ != null;
    }


}

public class YourService extends KiboRpcService {


    private Mat camMatrix;
    private Mat distCoeff;

    private Mat map1;
    private Mat map2;

    final String TAG = "SPACECAT";


    enum Param {
        // Camera distortion parameters for simulation and orbit cameras

        SIMULATION,

        ORBIT

    }

    enum Image {
        // Undistort camera for scanning, DISTORT will maintain original image

        UNDISTORT,

        DISTORT

    }

    protected void initCamera(Param param){

        camMatrix = new Mat(3, 3, CvType.CV_32F);
        distCoeff = new Mat(1, 5, CvType.CV_32F);

        if(param == Param.SIMULATION){

            float[] cameraArray =  {
                    344.173397f, 0.000000f, 630.793795f,
                    0.000000f, 344.277922f, 487.033834f,
                    0.000000f, 0.00000000f, 1.00000000f
            };

            camMatrix.put(0, 0, cameraArray);


            float[] distArray =  {
                    -0.152963f, 0.017530f, -0.001107f, -0.000210f, 0.000000f
            };

            distCoeff.put(0, 0, distArray);

        }else if(param == Param.ORBIT){

            float[] cameraArray = {

                    692.827528f, 0.000000f, 571.399891f,
                    0.000000f, 691.919547f, 504.956891f,
                    0.000000f, 0.000000f, 1.000000f

            };

            camMatrix.put(0, 0, cameraArray);

            float[] distArray = {

                    -0.312191f, 0.073843f, -0.000918f, 0.001890f, 0.000000f
            };

            distCoeff.put(0, 0, distArray);

        }

    }

    @Override
    protected void runPlan1() {

        api.judgeSendStart();

        //initCamera(Param.ORBIT);
        initCamera(Param.SIMULATION);

        //moveToWrapper(11, -4.90, 4.33, 0.500, 0.500, -0.500, 0.500);
        //moveToWrapper(11, -5.60, 4.33, 0.500, 0.500, -0.500, 0.500);

        moveToWrapper(11.3, -4.5, 4.95, 0, 0, 0.707, -0.707);
        moveToWrapper(10.7, -5.16, 4.42, 0, 0 ,1, 0);


        api.flashlightControlFront(0.025f);
        try {
            Thread.sleep(1000);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }

        scanBuffer(0);

        api.flashlightControlFront(0);

        moveToWrapper(10.7, -5.95, 4.42, 0, 0, 0.707, -0.707);
        moveToWrapper(10.455, -6.54, 4.42, 0, 0, 0.707, -0.707);
        moveToWrapper(11.06, -7.68, 5.47, 0.5, 0.5 ,0.5, -0.5);


        api.flashlightControlFront(0.025f);
        try {
            Thread.sleep(1000);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }

        scanBuffer(1);

        api.flashlightControlFront(0);


        moveToWrapper(11.2, -7.78, 4.85, 0, 0, 0.707, -0.707);
        moveToWrapper(11.2, -9, 4.85, 0, 0, 0.707, -0.707);

        if(QRData.PositionIsAvailable() && QRData.QuaternionIsAvailable()){

            moveToWrapper(QRData.getPosX(), QRData.getPosY(), QRData.getPosZ(),
                    QRData.getQuaX(), QRData.getQuaY(), QRData.getQuaZ(), QRData.getQuaW());

        }else if (QRData.PositionIsAvailable()){

            moveToWrapper(QRData.getPosX(), QRData.getPosY(), QRData.getPosZ(),
                    0, 0, 0.707, -0.707);

        }else{
            api.judgeSendFinishSimulation();
        }

        Mat offset_ar = decodeAR();

        if (offset_ar != null){

            //Offset data for lasering
            double offset_target_laser_x = 0.141421356; //0.1*sqrt(2) m
            double offset_target_laser_z = 0.141421356;
            double offset_camera_x = -0.0572;
            double offset_camera_z = 0.1111;
            double added_offset_x = -0.054;
            double added_offset_z = -0.075;

            //Acquiring translation vector as the error offset
            double[] offset_ar_largest = offset_ar.get(0, 0);

            double p3_posx = QRData.getPosX() + offset_ar_largest[0] + offset_target_laser_x + offset_camera_x + added_offset_x;
            double p3_posz = QRData.getPosZ() + offset_ar_largest[1] + offset_target_laser_z + offset_camera_z + added_offset_z;

            //move to target p3 for lasering (with plan B)
            double qr_pos_y = -9.58091874748;
            moveToWrapper(p3_posx, qr_pos_y , p3_posz , 0, 0, 0.707, -0.707);

            api.laserControl(true);
        }

        //api.judgeSendFinishISS();
        api.judgeSendFinishSimulation();
    }


    @Override
    protected void runPlan2() {

    }

    @Override
    protected void runPlan3() {

    }


    private void moveToWrapper(double pos_x, double pos_y, double pos_z,
                                  double qua_x, double qua_y, double qua_z,
                                  double qua_w) {

        final int LOOP_MAX = 100;
        final Point point = new Point(pos_x, pos_y, pos_z);
        final Quaternion quaternion = new Quaternion((float) qua_x, (float) qua_y,
                (float) qua_z, (float) qua_w);


        Log.i(TAG, "[0] Calling moveTo function ");
        long start = System.currentTimeMillis();

        Result result = api.moveTo(point, quaternion, true);

        if (result.getStatus() == Result.Status.BAD_SYNTAX) {

            api.moveTo(22, 22)

        }


        long end = System.currentTimeMillis();
        long elapsedTime = end - start;
        Log.i(TAG, "[0] moveTo finished in : " + elapsedTime/1000 + "seconds");
        Log.i(TAG, "[0] hasSucceeded : " + result.hasSucceeded());
        Log.i(TAG, "[0] getStatus : " + result.getStatus().toString());
        Log.i(TAG, "[0] getMessage : " + result.getMessage());

        int loopCounter = 1;
        while (!result.hasSucceeded() && loopCounter <= LOOP_MAX) {

            Log.i(TAG, "[" + loopCounter + "] " + "Calling moveTo function");
            start = System.currentTimeMillis();

            result = api.moveTo(point, quaternion, true);

            end = System.currentTimeMillis();
            elapsedTime = end - start;
            Log.i(TAG, "[" + loopCounter + "] " + "moveTo finished in : " + elapsedTime/1000 + "seconds");
            Log.i(TAG, "[" + loopCounter + "] " + "hasSucceeded : " + result.hasSucceeded());
            Log.i(TAG, "[" + loopCounter + "] " + "getStatus : " + result.getStatus().toString());
            Log.i(TAG, "[" + loopCounter + "] " + "getMessage : " + result.getMessage());

            ++loopCounter;

        }

    }


    private Boolean scanBuffer(int targetQR){

        List<Mat> QRBuffer = new ArrayList<Mat>();
        QRCodeReader reader = new QRCodeReader();

        for(int i = 0; i< 5; i ++){

            Mat navCam = api.getMatNavCam();

            if(i == 0){

                QRBuffer.add(navCam);

            }else{

                while(navCam == QRBuffer.get(i-1)){
                    navCam = api.getMatNavCam();
                }

                QRBuffer.add(navCam);

            }
        }

        map1 = new Mat();
        map2 = new Mat();
        initUndistortRectifyMap(camMatrix, distCoeff, new Mat(), camMatrix, new Size(1280,960), CV_16SC2, map1, map2 );

        for (Mat QR : QRBuffer){


            boolean success = decodeQR(targetQR, QR, 1280, 960, Image.UNDISTORT, "1280x960", reader);

            if (success){

                return true;

            }

        }

        int MAX_RETRY_TIMES = 10;
        int retryTimes = 1;

        while (retryTimes <= MAX_RETRY_TIMES) {

            Mat QR = api.getMatNavCam();

            boolean success = decodeQR(targetQR, QR, 1280, 960, Image.UNDISTORT, "1280x960", reader);

            if(success){

                return true;

            }

            retryTimes++;

        }

        return false;
    }

    private Boolean decodeQR(int targetQR, Mat inputImage, int resizeWidth, int resizeHeight,
                             Image image, String identifier, QRCodeReader reader) {

        reader.reset();

        final Map<DecodeHintType, ?> TRY_HARDER = Collections
                .singletonMap(DecodeHintType.TRY_HARDER, true);

        String qrString = null;

        Mat imageResizedMat = new Mat();
        Bitmap imageResizedBitmap;
        BinaryBitmap navcamBin;

        RGBLuminanceSource navcamLuminance;
        com.google.zxing.Result result;

        if (image == Image.UNDISTORT){

            Mat tmp = inputImage.clone();
            remap(tmp, inputImage, map1, map2, INTER_LINEAR, BORDER_CONSTANT);

        }

        Size size = new Size(resizeWidth, resizeHeight);

        Imgproc.resize(inputImage, imageResizedMat, size);

        //ImageSaver saver = new ImageSaver();
        //saver.save_image(imageResizedMat);

        //Convert navcam to bitmap
        imageResizedBitmap = matToBitmap(imageResizedMat);

        //Getting pixels data
        int width = imageResizedBitmap.getWidth();
        int height = imageResizedBitmap.getHeight();
        int[] navcamPixels = new int[width * height];

        //get the pixels out of bitmap
        imageResizedBitmap.getPixels(navcamPixels, 0, width, 0, 0, width, height);

        //get the luminance data
        navcamLuminance = new RGBLuminanceSource(width, height, navcamPixels);

        //convert to binary image
        navcamBin = new BinaryBitmap(new HybridBinarizer(navcamLuminance));

        //decoding QR result
        try {

            result = reader.decode(navcamBin, TRY_HARDER);
            qrString = result.getText();

        } catch (Exception e) {

            Log.i(TAG, identifier + "QR NOT FOUND");
            qrString = null;

        }

        if (qrString != null) {

            Log.i(TAG, identifier + qrString);

            //comment for testing image datas

            api.judgeSendDiscoveredQR(targetQR, qrString);

            if (targetQR == 0){

                QRData.storePosition(qrString);

            }else{

                QRData.storeQuaternion(qrString);
            }

            return true;

        }

        return false;
    }

    private Mat decodeAR(){
        Mat markerIds = new Mat();
        List<Mat> corners= new ArrayList<>();
        List<Mat> rejected= new ArrayList<>();
        Dictionary dictionary = Aruco.getPredefinedDictionary(Aruco.DICT_5X5_250);

        Mat rvec = new Mat();
        Mat tvec = new Mat();

        Mat nav_cam = api.getMatNavCam();
        //Mat dst = undistort_camera(nav_cam);

        DetectorParameters parameters = DetectorParameters.create();
        parameters.set_minDistanceToBorder(0);
        parameters.set_minMarkerPerimeterRate(0.01);
        parameters.set_maxMarkerPerimeterRate(4);
        parameters.set_polygonalApproxAccuracyRate(0.08);

        for (int i = 0; i < 10 && markerIds.cols() == 0 && markerIds.rows() == 0; i++){

            Aruco.detectMarkers(nav_cam/*dst*/, dictionary, corners, markerIds, parameters, rejected, camMatrix, distCoeff );

            if(markerIds.cols() != 0 && markerIds.rows() != 0){

                //Converting AR value from double to string
                double ARDouble = markerIds.get(0, 0)[0];
                int ARint = (int) ARDouble;
                QRData.Ar_Id = Integer.toString(ARint);

                Log.i(TAG, "AR Found : " + QRData.Ar_Id);
                api.judgeSendDiscoveredAR(QRData.Ar_Id);

                //Pose estimation
                Aruco.estimatePoseSingleMarkers(corners, 0.05f, camMatrix, distCoeff, rvec, tvec);

                /*
                Log.i("Corners", Integer.toString(corners.size()));
                Log.i("id", markerIds.dump());
                Log.i("rejected", Integer.toString(rejected.size()));
                */

                return tvec;
            }

            Log.i(TAG, "AR Not Found : " + (i + 1));

        }
        return null;
    }

    private Bitmap matToBitmap(Mat in){

        Bitmap bmp = null;
        try {

            bmp = Bitmap.createBitmap(in.cols(), in.rows(), Bitmap.Config.ARGB_8888);
            Utils.matToBitmap(in, bmp);

        }
        catch (CvException e){Log.d("Exception",e.getMessage());}

        return bmp;
    }

}


