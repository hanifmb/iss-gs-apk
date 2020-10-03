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
import static org.opencv.imgproc.Imgproc.warpAffine;

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

        //moveToWrapper(10.7, -5.16+offsetCamY_bodyFrame, 4.42-offsetCamZ_bodyFrame, 0, 0 ,1, 0);
        moveToWrapper(10.7, -5.57, 4.5, 0, 0 ,1, 0);


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

        //moveToWrapper(11.06-offsetCamY_bodyFrame, -7.68-offsetCamZ_bodyFrame, 5.47, 0.5, 0.5 ,0.5, -0.5);
        moveToWrapper(11.1, -7.98, 5.47, -0.5, -0.5 ,-0.5, 0.5);

        api.flashlightControlFront(0.025f);

        try {
            Thread.sleep(1000);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }

        scanBuffer(1);

        api.flashlightControlFront(0);

        moveToWrapper(11.06, -7.78, 4.85, 0, 0, 0.707, -0.707);
        moveToWrapper(11.2, -9, 4.85, 0, 0, 0.707, -0.707);

        if(QRData.PositionIsAvailable() && QRData.QuaternionIsAvailable()){


            moveToWrapper(QRData.getPosX(), QRData.getPosY(), QRData.getPosZ(),
                    QRData.getQuaX(), QRData.getQuaY(), QRData.getQuaZ(), QRData.getQuaW());


        }else{
            //api.judgeSendFinishISS();
            api.judgeSendFinishSimulation();
        }

        /*
        Tal = Laser target transformation matrix w.r.t AR Tag
        Tcr = AR Tag transformation matrix w.r.t camera
        Tac = Camera transformation matrix w.r.t astrobee
        Twa = Astrobee transformation matrix w.r.t ISS(world)

        Twr = AR Tag transformation matrix w.r.t ISS
        Twl = Laser target transformation matrix w.r.t ISS

        Twl = Twa * Tac * Tcr * Tal
         */

        List<Mat> offsetAR = decodeAR();

        if (offsetAR != null) {
            double[] TrlElements = {
                    /*
                    P = [x,
                         y,
                         z];  P is the 20 cm offset between AR and Laser target

                    Tac =
                    [ rotMal, P ]
                    [    0  , 1 ]

                    rotMal is the 0 rotation -> object on the same plane
                     */

                    1.0000000,  0.0000000,  0.0000000, -0.1414213,
                    0.0000000,  1.0000000,  0.0000000, -0.1414213,
                    0.0000000,  0.0000000,  1.0000000, 0.0000000,
                    0.0000000,  0.0000000,  0.0000000, 1.0000000

            };
            Mat Trl = new Mat(4, 4, CV_64F);
            Trl.put(0, 0, TrlElements);

            Mat rvec = offsetAR.get(0);
            Mat tvec = offsetAR.get(1);

            Log.d(TAG, "rvec" + rvec.dump());
            Log.d(TAG, "tvec" + tvec.dump());

            Mat rotMcr = new Mat(3, 3, CV_64F);

            //Converting rotation vector to rotation matrix, AR tag orientation
            Calib3d.Rodrigues(rvec, rotMcr);
            Log.d(TAG, "ROTATION MATRIX CAMERA-AR");
            Log.d(TAG, rotMcr.dump());

            Mat Tcr = new Mat(4, 4, CV_64F);

            //Building AR transformation matrix w.r.t navigation cam
            double[] TcrElements = {
                    /*
                    P = [x,
                         y,
                         z];  P is the AR tag translation vector

                    Tac =
                    [ rotMcr, P ]
                    [    0  , 1 ]
                     */

                    rotMcr.get(0,0)[0], rotMcr.get(0,1)[0], rotMcr.get(0,2)[0], tvec.get(0, 0)[0],
                    rotMcr.get(1,0)[0], rotMcr.get(1,1)[0], rotMcr.get(1,2)[0], tvec.get(0, 0)[1],
                    rotMcr.get(2,0)[0], rotMcr.get(2,1)[0], rotMcr.get(2,2)[0], tvec.get(0, 0)[2],
                    0.00000, 0.00000, 0.00000, 1.00000
            };

            Tcr.put(0,0, TcrElements);

            Log.d(TAG, "TRANSFORMATION MATRIX CAMERA-AR");
            Log.d(TAG, Tcr.dump());

            //Building camera transformation matrix w.r.t astrobee
            double[] TacElements = {
                    /*
                    P = [x,
                         y,
                         z];  P is the astrobee camera offset

                    Tac =
                    [ rotMac, P ]
                    [   0   , 1 ]
                     */

                    0.0000000,  0.0000000,  1.000000, 0.1177,
                    1.0000000,  0.0000000,  0.0000000, -0.0422,
                    0.0000000,  1.0000000,  0.0000000, -0.0826,
                    0.0000000,  0.0000000,  0.0000000, 1.0000

            };

            Mat Tac = new Mat(4, 4, CV_64F);
            Tac.put(0, 0, TacElements);

            //Calculating rotation matrix of astrobee w.r.t ISS
            Kinematics kinematics = api.getTrustedRobotKinematics(5);

            double posX, posY, posZ;
            float quaX, quaY, quaZ, quaW;

            if(kinematics != null){

                posX = kinematics.getPosition().getX();
                posY = kinematics.getPosition().getY();
                posZ = kinematics.getPosition().getZ();

                quaX = kinematics.getOrientation().getX();
                quaY = kinematics.getOrientation().getY();
                quaZ = kinematics.getOrientation().getZ();
                quaW = kinematics.getOrientation().getW();

            }else{

                posX = QRData.getPosX();
                posY = QRData.getPosY();
                posZ = QRData.getPosZ();

                quaX = (float)QRData.getQuaX();
                quaY = (float)QRData.getQuaY();
                quaZ = (float)QRData.getQuaZ();
                quaW = (float)QRData.getQuaW();

            }

            Log.d(TAG, "CURRENT POSITION OF ASTROBEE");
            Log.d(TAG, quaX + " " + quaY + " " + quaZ + " " + quaW);
            Log.d(TAG, posX + " " + posY + " " + posZ);

            Quaternion quaternion = new Quaternion(quaX, quaY, quaZ, quaW);
            Mat rotMwa = quatToMatrix2(quaternion);

            Log.d(TAG, "ROTATION MATRIX ISS-ASTROBEE");
            Log.d(TAG, rotMwa.dump());

            double det = Core.determinant(rotMwa);
            Log.d(TAG, "DETERMINANT = " + det);

            //Building transformation matrix ISS w.r.t ASTROBEE
            double[] TwaElements = {
                    /*
                    P = [x,
                         y,
                         z];  P is the astrobee global position (w.r.t ISS)

                    Tac =
                    [ rotMwa, P ]
                    [   0   , 1 ]
                     */

                    rotMwa.get(0,0)[0], rotMwa.get(0,1)[0], rotMwa.get(0,2)[0], posX,
                    rotMwa.get(1,0)[0], rotMwa.get(1,1)[0], rotMwa.get(1,2)[0], posY,
                    rotMwa.get(2,0)[0], rotMwa.get(2,1)[0], rotMwa.get(2,2)[0], posZ,
                    0.00000, 0.00000, 0.00000, 1.00000
            };

            Mat Twa = new Mat(4, 4, CV_64F);
            Twa.put(0, 0, TwaElements);

            Log.d(TAG, "TRANSFORMATION MATRIX ISS-ASTROBEE");
            Log.d(TAG, Twa.dump());

            //Calculating Twr = Twa * Tac * Tcr
            Mat Twr = new Mat(4, 4, CV_64F);
            Mat Twc = new Mat(4, 4, CV_64F);
            Mat Twl = new Mat(4, 4, CV_64F);

            Core.gemm(Twa, Tac, 1, new Mat(), 0, Twc, 0);
            Core.gemm(Twc, Tcr, 1, new Mat(), 0, Twr, 0);
            Core.gemm(Twr, Trl, 1, new Mat(), 0, Twl, 0);

            Log.d(TAG, "TRANSFORMATION MATRIX ISS-AR Tag");
            Log.d(TAG, Twr.dump());

            Log.d(TAG, "TRANSFORMATION MATRIX ISS-Laser Tag");
            Log.d(TAG, Twl.dump());

            double targetX = Twr.get(0, 3)[0];
            double targetZ = Twr.get(2, 3)[0];

            targetX += 0.1414;
            targetZ += 0.1414;

            Log.d(TAG, "Laser based on world offset");
            Log.d(TAG, targetX + " " + targetZ);

            targetX -= 0.0572;
            targetZ += 0.1111;

            moveToWrapper(targetX, posY, targetZ,0, 0, 0.707, -0.707);

            api.laserControl(true);

        }
        //api.judgeSendFinishISS();
        api.judgeSendFinishSimulation();
    }


    @Override
    protected void runPlan2() {

        initCamera(Param.SIMULATION);

        map1 = new Mat();
        map2 = new Mat();
        initUndistortRectifyMap(camMatrix, distCoeff, new Mat(), camMatrix, new Size(1280,960), CV_16SC2, map1, map2 );

        scanARImages();


    }

    @Override
    protected void runPlan3() {


        api.judgeSendStart();
        initCamera(Param.SIMULATION);

        map1 = new Mat();
        map2 = new Mat();
        initUndistortRectifyMap(camMatrix, distCoeff, new Mat(), camMatrix, new Size(1280,960), CV_16SC2, map1, map2 );

        Mat img = imread(Environment.getExternalStorageDirectory().getAbsolutePath() +"/hanif2/" + "frame0450.jpg");
        List<Mat> offsetAR = decodeAR();

        if (offsetAR != null) {
            double[] TrlElements = {
                    /*
                    P = [x, y, z];  P is the 20 cm offset between AR and Laser target

                    Tac =
                    [ rotMal, P ]
                    [    0  , 1 ]

                    rotMal is the 0 rotation -> object on the same plane
                     */

                    1.0000000,  0.0000000,  0.0000000, 0.1414213,
                    0.0000000,  1.0000000,  0.0000000, -0.1414213,
                    0.0000000,  0.0000000,  1.0000000, 0.0000000,
                    0.0000000,  0.0000000,  0.0000000, 1.0000000

            };
            Mat Trl = new Mat(4, 4, CV_64F);
            Trl.put(0, 0, TrlElements);

            Mat rvec = offsetAR.get(0);
            Mat tvec = offsetAR.get(1);

            Log.d(TAG, "rvec" + rvec.dump());
            Log.d(TAG, "tvec" + tvec.dump());

            Mat rotMcr = new Mat(3, 3, CV_64F);

            //Converting rotation vector to rotation matrix, AR tag orientation
            Calib3d.Rodrigues(rvec, rotMcr);
            Log.d(TAG, "ROTATION MATRIX CAMERA-AR");
            Log.d(TAG, rotMcr.dump());

            Mat Tcr = new Mat(4, 4, CV_64F);

            //Building AR transformation matrix w.r.t AR tag
            double[] TcrElements = {
                    /*
                    P = [x, y, z];  P is the AR tag translation vector

                    Tac =
                    [ rotMcr, P ]
                    [    0  , 1 ]
                     */

                    rotMcr.get(0,0)[0], rotMcr.get(0,1)[0], rotMcr.get(0,2)[0], tvec.get(0, 0)[0],
                    rotMcr.get(1,0)[0], rotMcr.get(1,1)[0], rotMcr.get(1,2)[0], tvec.get(0, 0)[1],
                    rotMcr.get(2,0)[0], rotMcr.get(2,1)[0], rotMcr.get(2,2)[0], tvec.get(0, 0)[2],
                    0.00000, 0.00000, 0.00000, 1.00000
            };

            Tcr.put(0,0, TcrElements);

            Log.d(TAG, "TRANSFORMATION MATRIX CAMERA-AR");
            Log.d(TAG, Tcr.dump());

            //Building robot transformation matrix w.r.t camera
            double[] TacElements = {
                    /*
                    P = [x, y, z];  P is the astrobee camera offset

                    Tac =
                    [ rotMac, P ]
                    [   0   , 1 ]
                     */

                    0.0000000,  0.0000000,  1.000000, 0.1177,
                    1.0000000,  0.0000000,  0.0000000, -0.0422,
                    0.0000000,  1.0000000,  0.0000000, -0.0826,
                    0.0000000,  0.0000000,  0.0000000, 1.0000

            };

            Mat Tac = new Mat(4, 4, CV_64F);
            Tac.put(0, 0, TacElements);

            //Calculating rotation matrix of astrobee w.r.t ISS

            double posX = 10.52938314321329;
            double posY = -9.45293840397371;
            double posZ = 4.993802269474;

            float quaX = -0.6849197f;
            float quaY = 0.000014f;
            float quaZ = -0.6849197f;
            float quaW = 0.72858155f;

            Log.d(TAG, "CURRENT POSITION OF ASTROBEE");
            Log.d(TAG, quaX + " " + quaY + " " + quaZ + " " + quaW);
            Log.d(TAG, posX + " " + posY + " " + posZ);

            Quaternion quaternion = new Quaternion(quaX, quaY, quaZ, quaW);
            Mat rotMwa = quatToMatrix2(quaternion);

            Log.d(TAG, "ROTATION MATRIX ISS-ASTROBEE");
            Log.d(TAG, rotMwa.dump());

            double det = Core.determinant(rotMwa);
            Log.d(TAG, "DETERMINANT = " + det);

            //Building transformation matrix ISS-ASTROBEE
            double[] TwaElements = {
                    /*
                    P = [x, y, z];  P is the astrobee global position (w.r.t ISS)

                    Tac =
                    [ rotMwa, P ]
                    [   0   , 1 ]
                     */

                    rotMwa.get(0,0)[0], rotMwa.get(0,1)[0], rotMwa.get(0,2)[0], posX,
                    rotMwa.get(1,0)[0], rotMwa.get(1,1)[0], rotMwa.get(1,2)[0], posY,
                    rotMwa.get(2,0)[0], rotMwa.get(2,1)[0], rotMwa.get(2,2)[0], posZ,
                    0.00000, 0.00000, 0.00000, 1.00000
            };

            Mat Twa = new Mat(4, 4, CV_64F);
            Twa.put(0, 0, TwaElements);

            Log.d(TAG, "TRANSFORMATION MATRIX ISS-ASTROBEE");
            Log.d(TAG, Twa.dump());

            //Calculating Twr = Twa * Tac * Tcr
            Mat Twr = new Mat(4, 4, CV_64F);
            Mat Twc = new Mat(4, 4, CV_64F);
            Mat Twl = new Mat(4, 4, CV_64F);

            Core.gemm(Twa, Tac, 1, new Mat(), 0, Twc, 0);
            Core.gemm(Twc, Tcr, 1, new Mat(), 0, Twr, 0);
            Core.gemm(Twr, Trl, 1, new Mat(), 0, Twl, 0);

            Log.d(TAG, "TRANSFORMATION MATRIX ISS-AR Tag");
            Log.d(TAG, Twr.dump());

            Log.d(TAG, "TRANSFORMATION MATRIX ISS-Laser Tag");
            Log.d(TAG, Twl.dump());

            double targetX = Twl.get(0, 3)[0];
            double targetZ = Twl.get(2, 3)[0];

            targetX -= 0.0572;
            targetZ += 0.1111;

            moveToWrapper(targetX, posY, targetZ,0, 0, 0.707, -0.707);

            api.laserControl(true);

            api.judgeSendFinishSimulation();

        }
    }


    private Boolean moveToWrapper(double pos_x, double pos_y, double pos_z,
                                  double qua_x, double qua_y, double qua_z,
                                  double qua_w) {

        final int LOOP_MAX = 50;
        final Point point = new Point(pos_x, pos_y, pos_z);
        final Quaternion quaternion = new Quaternion((float) qua_x, (float) qua_y,
                (float) qua_z, (float) qua_w);


        Log.i(TAG, "[0] Calling moveTo function ");
        Log.i(TAG, pos_x + " " + pos_y + " " + pos_z);
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

        return true;

    }


    private Boolean scanBuffer(int targetQR){

        List<Mat> QRBuffer = new ArrayList<Mat>();
        QRCodeReader reader = new QRCodeReader();

        for(int i = 0; i< 5; i ++){

            Mat navCam = getNavcamMat();

            if(i == 0){

                QRBuffer.add(navCam);

            }else{

                int counter = 0;
                while(navCam == QRBuffer.get(i-1) && counter < 5){

                    try {
                        Thread.sleep(100);
                    } catch (InterruptedException e) {
                        e.printStackTrace();
                    }

                    navCam = getNavcamMat();

                    counter++;

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

        int MAX_RETRY_TIMES = 240;
        int retryTimes = 1;

        while (retryTimes <= MAX_RETRY_TIMES) {

            Mat QR = getNavcamMat();

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

        DetectorParameters parameters = DetectorParameters.create();

        for (int i = 0; i < 240; i++){

            Mat nav_cam = getNavcamMat();
            /*
            Mat tmp = nav_cam.clone();
            remap(tmp, nav_cam, map1, map2, INTER_LINEAR, BORDER_CONSTANT);
             */

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

    private Mat getNavcamMat(){

        Mat image = api.getMatNavCam();

        int counter = 0;
        while (image == null && counter <5) {

            try {
                Thread.sleep(300);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }

            image = api.getMatNavCam();
            counter++;

        }

        return image;

    }

    /*

    for real world image testing purpose

     */

    private void scanSDImages(){

        File filePath = new File(Environment.getExternalStorageDirectory().getAbsolutePath() +"/100QR/");

        File[] listingAllFiles = filePath.listFiles();
        List<File> allJpg = imageFileUtils.iterateOverFiles(listingAllFiles);

        for (File file : allJpg) {
            String fileAbsPath = file.getAbsolutePath();
            String absPath2 = Environment.getExternalStorageDirectory().getAbsolutePath();

            Log.d(TAG, fileAbsPath);
            Log.d(TAG, absPath2);

            Mat img = imread(fileAbsPath);

            QRCodeReader reader = new QRCodeReader();
            decodeQR(0, img, 1280, 960, Image.DISTORT, "SCAN_RES", reader);
            //decodeWithZbar(0, img);

        }

    }

    private void scanARImages(){

        File filePath = new File(Environment.getExternalStorageDirectory().getAbsolutePath() +"/100AR/");

        File[] listingAllFiles = filePath.listFiles();
        List<File> allJpg = imageFileUtils.iterateOverFiles(listingAllFiles);

        for (File file : allJpg) {
            String fileAbsPath = file.getAbsolutePath();
            String absPath2 = Environment.getExternalStorageDirectory().getAbsolutePath();

            Log.d(TAG, fileAbsPath);
            Log.d(TAG, absPath2);

            Mat img = imread(fileAbsPath);

            decodeAR2(img);

            //decodeWithZbar(0, img);

        }

    }

    private List<Mat> decodeAR2(Mat nav_cam){

        Mat markerIds = new Mat();
        List<Mat> corners= new ArrayList<>();
        List<Mat> rejected= new ArrayList<>();
        Dictionary dictionary = Aruco.getPredefinedDictionary(Aruco.DICT_5X5_250);

        Mat rvec = new Mat();
        Mat tvec = new Mat();

        DetectorParameters parameters = DetectorParameters.create();

            /*
            Mat tmp = nav_cam.clone();
            remap(tmp, nav_cam, map1, map2, INTER_LINEAR, BORDER_CONSTANT);
             */

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

        Log.i(TAG, "AR NOT FOUND");
        return null;
    }

}

