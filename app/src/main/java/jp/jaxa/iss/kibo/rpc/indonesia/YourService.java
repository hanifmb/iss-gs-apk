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
import org.opencv.calib3d.Calib3d;
import org.opencv.core.Core;
import org.opencv.core.CvException;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;

import static org.opencv.core.Core.BORDER_CONSTANT;
import static org.opencv.core.Core.addWeighted;
import static org.opencv.core.Core.determinant;
import static org.opencv.core.CvType.CV_16SC2;
import static org.opencv.core.CvType.CV_32F;
import static org.opencv.core.CvType.CV_64F;
import static org.opencv.imgcodecs.Imgcodecs.CV_IMWRITE_EXR_TYPE;
import static org.opencv.imgcodecs.Imgcodecs.imread;
import static org.opencv.imgproc.Imgproc.GaussianBlur;
import static org.opencv.imgproc.Imgproc.INTER_LINEAR;
import static org.opencv.imgproc.Imgproc.filter2D;
import static org.opencv.imgproc.Imgproc.initUndistortRectifyMap;
import static org.opencv.imgproc.Imgproc.remap;

import java.io.File;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.List;
import java.util.Map;

/**
 * Class meant to handle commands from the Ground Data System and execute them in Astrobee
 */

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

        }else{
            api.judgeSendFinishSimulation();
        }

        List<Mat> offsetAR = decodeAR();

        if (offsetAR != null){

            //Offset data for lasering
            double offset_target_laser_x = 0.141421356; //0.1*sqrt(2) m
            double offset_target_laser_z = 0.141421356;
            double offset_camera_x = -0.0572;
            double offset_camera_z = 0.1111;
            double added_offset_x = -0.054;
            double added_offset_z = -0.075;

            //Acquiring translation vector as the error offset
            Mat OffsetARLargest1 = offsetAR.get(1);
            double[] offsetArLargest = OffsetARLargest1.get(0, 0);

            /* quaternion orientation restriction

            -phi < z < 0
            -phi/2 < y < phi/2
            -phi < x x < phi

             quaternion to euler angle sequence
             z -> y -> x
             */

            double p3_posx = QRData.getPosX() + offsetArLargest[0] + offset_target_laser_x + offset_camera_x + added_offset_x;
            double p3_posz = QRData.getPosZ() + offsetArLargest[1] + offset_target_laser_z + offset_camera_z + added_offset_z;

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
        api.judgeSendStart();
        initCamera(Param.SIMULATION);

        List<Mat> offsetAR = decodeAR();

        if (offsetAR != null) {

            //Offset data for lasering

            //Acquiring translation vector as the error offset

            //Calculating Rotation Matrix (R21)
            Mat rvec = offsetAR.get(0);
            Mat tvec = offsetAR.get(1);

            Log.d(TAG, "rvec" + rvec.dump());
            Log.d(TAG, "tvec" + tvec.dump());

            Mat rotM = new Mat(3, 3, CV_64F);

            Calib3d.Rodrigues(rvec, rotM);
            Log.d(TAG, "ROTATION MATRIX T21");
            Log.d(TAG, rotM.dump());

            //float det = (float) Core.determinant(rotM);
            //Log.d(TAG, "DETERMINANT" + det);

            Mat T21 = new Mat(4, 4, CV_64F);

            //Arranging T21
            double[] T21Scalar = {

                    rotM.get(0,0)[0], rotM.get(0,1)[0], rotM.get(0,2)[0], tvec.get(0, 0)[0],
                    rotM.get(1,0)[0], rotM.get(1,1)[0], rotM.get(1,2)[0], tvec.get(0, 0)[1],
                    rotM.get(2,0)[0], rotM.get(2,1)[0], rotM.get(2,2)[0], tvec.get(0, 0)[2],
                    0, 0, 0, 1
            };

            T21.put(0,0, T21Scalar);

            Log.d(TAG, "TRANSFORMATION MATRIX T21");
            Log.d(TAG, T21.dump());

            //Calculating transformation matrix between robot and camera
            double[] TrcScalar = {

                    0.0000000,  0.0000000,  1.000000, 0.1177,
                    1.0000000,  0.0000000,  0.0000000, -0.0422,
                    0.0000000,  1.0000000,  0.0000000, -0.0826,
                    0.0000000,  0.0000000,  0.0000000, 1.0000

            };

            Mat Trc = new Mat(4, 4, CV_64F);
            Trc.put(0, 0, TrcScalar);

            //Calculating rotation matrix (R32)
            Kinematics kinematics = api.getTrustedRobotKinematics();

            if(kinematics.getConfidence() == Kinematics.Confidence.GOOD){
                double posX = kinematics.getPosition().getX();
                double posY = kinematics.getPosition().getY();
                double posZ = kinematics.getPosition().getZ();

                float quaX = kinematics.getOrientation().getX();
                float quaY = kinematics.getOrientation().getY();
                float quaZ = kinematics.getOrientation().getZ();
                float quaW = (float)Math.sqrt(quaX*quaX + quaY*quaY + quaZ*quaZ);

                Quaternion quaternion = new Quaternion(quaX, quaY, quaZ, quaW);
                Mat rotM32 = quatToMatrix2(quaternion);

                double det = Core.determinant(rotM32);
                Log.d(TAG, "DETERMINANT" + det);

                //Rearranging transformation matrix (T32)
                double[] T32Scalar = {

                        rotM32.get(0,0)[0], rotM32.get(0,1)[0], rotM32.get(0,2)[0], posX,
                        rotM32.get(1,0)[0], rotM32.get(1,1)[0], rotM32.get(1,2)[0], posY,
                        rotM32.get(2,0)[0], rotM32.get(2,1)[0], rotM32.get(2,2)[0], posZ,
                        0, 0, 0, 1
                };

                Mat T32 = new Mat(4, 4, CV_64F);
                T32.put(0, 0, T32Scalar);

                Log.d(TAG, "TRANSFORMATION MATRIX T32");
                Log.d(TAG, T32.dump());

                //Calculating T31 = T32 * T21
                Mat T31 = new Mat(4, 4, CV_64F);
                Mat Ttemp = new Mat(4, 4, CV_64F);

                Core.gemm(T32, Trc, 1, new Mat(), 0, Ttemp, 0);
                Core.gemm(Ttemp, T21, 1, new Mat(), 0, T31, 0);


                Log.d(TAG, "TRANSFORMATION MATRIX T31");
                Log.d(TAG, T31.dump());
                Log.d(TAG, "DETERMINANT" + Core.determinant(T31));

                double targetX = T31.get(0, 3)[0];
                double targetY = posY;
                double targetZ = T31.get(2, 3)[0];

                moveToWrapper(targetX, targetY, targetZ,0, 0, 0.707, -0.707);
                api.laserControl(true);

            }


            /* quaternion orientation restriction

            -phi < z < 0
            -phi/2 < y < phi/2
            -phi < x x < phi

             quaternion to euler angle sequence
             z -> y -> x
             */

            //move to target p3 for lasering (with plan B)
            double qr_pos_y = -9.58091874748;


            api.laserControl(true);
        }
    }

    @Override
    protected void runPlan3() {
        moveToWrapper(11, -5.60, 4.32,
                0.500, 0.500, -0.500, 0.500);
        moveToWrapper(11.37, -5.70, 4.5, 0, 0, 0, 1);

        moveToWrapper(11, -6, 5.45,  0.707, 0,  0.707, 0);
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

    private List<Mat> decodeAR(){
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

                List<Mat> transform = new ArrayList<Mat>();
                transform.add(rvec);
                transform.add(tvec);
                return transform;


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
        catch (CvException e){Log.d("Exception", e.getMessage());}

        return bmp;
    }


    public final Mat quatToMatrix(Quaternion q){
        double sqw = q.getW()*q.getW();
        double sqx = q.getW()*q.getX();
        double sqy = q.getY()*q.getY();
        double sqz = q.getZ()*q.getZ();

        // invs (inverse square length) is only required if quaternion is not already normalised
        double invs = 1.0f;
        double m00 = ( sqx - sqy - sqz + sqw)*invs ; // since sqw + sqx + sqy + sqz =1/invs*invs
        double m11 = (-sqx + sqy - sqz + sqw)*invs ;
        double m22 = (-sqx - sqy + sqz + sqw)*invs ;

        double tmp1 = q.getX()*q.getY();
        double tmp2 = q.getZ()*q.getW();
        double m10 = 2.0 * (tmp1 + tmp2)*invs ;
        double m01 = 2.0 * (tmp1 - tmp2)*invs ;

        tmp1 = q.getX()*q.getZ();
        tmp2 = q.getY()*q.getW();
        double m20 = 2.0 * (tmp1 - tmp2)*invs ;
        double m02 = 2.0 * (tmp1 + tmp2)*invs ;
        tmp1 = q.getY()*q.getZ();
        tmp2 = q.getX()*q.getW();
        double m21 = 2.0 * (tmp1 + tmp2)*invs ;
        double m12 = 2.0 * (tmp1 - tmp2)*invs ;

        Mat rotationMatrix = new Mat(3, 3, CV_64F);
        double[] rotM = {
             m00, m01, m02,
             m10, m11, m12,
             m20, m21, m22
        };
        rotationMatrix.put(0,0, rotM);

        Log.d(TAG, "ROTATION MATRIX");
        Log.d(TAG, rotationMatrix.dump());

        return rotationMatrix;

    }

    private Mat quatToMatrix2(Quaternion q){

        double qw = q.getW();
        double qx = q.getX();
        double qy = q.getY();
        double qz = q.getZ();

        Mat rotM = new Mat(3, 3, CV_64F);

        double[] rotMElements = {

            1.0 - 2.0*qy*qy - 2.0*qz*qz, 2.0*qx*qy - 2.0*qz*qw, 2.0*qx*qz + 2.0*qy*qw,
            2.0*qx*qy + 2.0*qz*qw, 1.0 - 2.0*qx*qx - 2.0*qz*qz, 2.0*qy*qz - 2.0*qx*qw,
            2.0*qx*qz - 2.0*qy*qw, 2.0*qy*qz + 2.0*qx*qw, 1.0 - 2.0*qx*qx - 2.0*qy*qy

        };

        rotM.put(0, 0, rotMElements);
        return rotM;

    }


}

