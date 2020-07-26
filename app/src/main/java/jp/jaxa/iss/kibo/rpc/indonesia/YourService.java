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

import net.sourceforge.zbar.ImageScanner;
import net.sourceforge.zbar.Image;
import net.sourceforge.zbar.Symbol;
import net.sourceforge.zbar.SymbolSet;
import net.sourceforge.zbar.Config;

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

    static double getQuaW(){

        return 1.0 - Math.pow(Double.parseDouble(quaX), 2.0)
                - Math.pow(Double.parseDouble(quaY), 2.0)
                - Math.pow(Double.parseDouble(quaZ), 2.0);

    }

    static Boolean PositionIsAvailable(){
        return posX != null && posY != null && posZ != null;
    }

    static Boolean QuaternionIsAvailable(){
        return quaX != null && quaY != null && quaZ != null;
    }

}

public class YourService extends KiboRpcService {

    static {
        System.loadLibrary("iconv");
    }

    private Mat camMatrix;
    private Mat distCoeff;
    final String TAG = "SPACECAT";

    //undistort

    private Mat map1;
    private Mat map2;

    //undistort


    enum Camera {
        // Choose between navigation cam or dock cam for QR Scanning

        NAVCAM,

        DOCKCAM

    }

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

        //init rectify map once
        map1 = new Mat();
        map2 = new Mat();
        initUndistortRectifyMap(camMatrix, distCoeff, new Mat(), camMatrix, new Size(1280,960), CV_16SC2, map1, map2 );


        //moveToWrapper(11, -5.10, 4.33, 0.500, 0.500, -0.500, 0.500);
        //moveToWrapper(11, -5.60, 4.33, 0.500, 0.500, -0.500, 0.500);

        moveToWrapper(11.2  , -5.16, 4.42, 0, 0 ,1, 0);
        moveToWrapper(10.72, -5.16, 4.42, 0, 0 ,1, 0);

        //decodeQRWithCam(0, Camera.NAVCAM, Image.UNDISTORT, 1280, 960, "1280x960_UNDISTORT");
        //decodeQRCode(0, true);
        //decodeBitmap(0);
        //decodeWithZbar(0, true);
        scanBuffer(0);

        api.judgeSendFinishSimulation();

        moveToWrapper(10.7, -5.95, 4.42, 0, 0, 0.707, -0.707);
        moveToWrapper(10.455, -6.54, 4.42, 0, 0, 0.707, -0.707);
        moveToWrapper(11.06, -7.68, 5.4, 0.5, -0.5 ,0.5, 0.5);

        //decodeQRWithCam(1, Camera.NAVCAM, Image.UNDISTORT, 1280, 960, "1280x960_UNDISTORT");
        //decodeQRCode(1, false);
        //decodeBitmap(1);
        //decodeWithZbar(0, true);
        scanBuffer(1);

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


        //api.judgeSendFinishISS();
        api.judgeSendFinishSimulation();
    }


    @Override
    protected void runPlan2() {
        initCamera(Param.SIMULATION);

        map1 = new Mat();
        map2 = new Mat();
        initUndistortRectifyMap(camMatrix, distCoeff, new Mat(), camMatrix, new Size(1280,960), CV_16SC2, map1, map2 );

        scanSDImages();

    }

    @Override
    protected void runPlan3() {

    }


    private void moveToWrapper(double pos_x, double pos_y, double pos_z,
                                  double qua_x, double qua_y, double qua_z,
                                  double qua_w) {

        final int LOOP_MAX = 50;
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

    private boolean decodeQRWithCam(int targetQR, Camera camera, Image image, int resizeWidth, int resizeHeight, String identifier){

        Mat inputImage;

        int MAX_RETRY_TIMES = 5;
        int retryTimes = 1;

        while (retryTimes <= MAX_RETRY_TIMES) {

            if (camera == Camera.DOCKCAM) {
                inputImage = api.getMatDockCam();
            } else {
                inputImage = api.getMatNavCam();
            }


            QRCodeReader reader = new QRCodeReader();
            boolean success = decodeQR(targetQR, inputImage, resizeWidth, resizeHeight, image, identifier, reader);

            if(success){
                return true; 
            }

            retryTimes++;

        }

        return false;
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

        for (Mat QR : QRBuffer){

            boolean success = decodeQR(targetQR, QR, 1280, 960, Image.UNDISTORT, "BISMILLAH", reader);

            if (success){

                return true;

            }

        }


        int MAX_RETRY_TIMES = 10;
        int retryTimes = 1;

        while (retryTimes <= MAX_RETRY_TIMES) {

            Mat QR = api.getMatNavCam();

            boolean success = decodeQR(targetQR, QR, 1280, 960, Image.UNDISTORT, "BISMILLAH", reader);

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

            inputImage = undistortCamera(inputImage, camMatrix, distCoeff);

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

    private void scanSDImages(){

        File filePath = new File(Environment.getExternalStorageDirectory().getAbsolutePath() +"/hanif/");

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


    private Mat undistortCamera(Mat img, Mat camMatrix, Mat distCoeff){

        Mat dst = new Mat(img.rows(), img.cols(), img.type());
        Imgproc.undistort(img, dst, camMatrix, distCoeff);
        return dst;

    }

    private Mat decodeAR(){
        Mat markerIds = new Mat();
        List<Mat> corners= new ArrayList<>();
        List<Mat> rejected= new ArrayList<>();
        Dictionary dictionary = Aruco.getPredefinedDictionary(Aruco.DICT_5X5_250);

        Mat camMatrix = new Mat(3, 3, CvType.CV_32F);
        Mat distCoeff = new Mat(1, 5, CvType.CV_32F);

        Mat rvec = new Mat();
        Mat tvec = new Mat();

        camMatrix.put(0,0, new float[]{344.173397f, 0.000000f, 630.793795f});
        camMatrix.put(1,0, new float[]{0.000000f, 344.277922f, 487.033834f});
        camMatrix.put(2,0, new float[]{0.000000f, 0.000000f, 1.000000f});
        distCoeff.put(0,0, new float[]{-0.152963f, 0.017530f, -0.001107f, -0.000210f, 0.000000f});

        Mat nav_cam = api.getMatNavCam();
        //Mat dst = undistort_camera(nav_cam);

        DetectorParameters parameters = DetectorParameters.create();
        parameters.set_minDistanceToBorder(0);
        parameters.set_minMarkerPerimeterRate(0.01);
        parameters.set_maxMarkerPerimeterRate(4);
        parameters.set_polygonalApproxAccuracyRate(0.08);

        for (int i = 0; i < 5 && markerIds.cols() == 0 && markerIds.rows() == 0; i++){

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

    /*
    private Boolean decodeQRCode(int target_qr, boolean useNavcam) {

        QRCodeReader reader = new QRCodeReader();

        Mat navcam_mat;
        Mat navcam_mat_undistort = new Mat();
        Bitmap navcam_bit_undistort;

        Size sz = new Size(1280,960);

        //get camera data once to initialize width, height and pixels
        BinaryBitmap navcam_bin;
        com.google.zxing.Result result;
        RGBLuminanceSource navcam_luminance;

        int MAX_RETRY_TIMES = 7;
        int retryTimes = 0;

        String qr_string = "";

        while (qr_string == "" && retryTimes < MAX_RETRY_TIMES) {

            try{

                reader.reset();

                //get navcam matrix
                if(useNavcam){
                    navcam_mat = api.getMatNavCam();
                }else{
                    navcam_mat = api.getMatDockCam();
                }


                //navcam_mat_undistort = navcam_mat;

                Imgproc.resize(navcam_mat, navcam_mat_undistort, sz );

                //Convert navcam to bitmap
                navcam_bit_undistort = matToBitmap(navcam_mat_undistort);

                //Getting pixels data
                int width = navcam_bit_undistort.getWidth();
                int height = navcam_bit_undistort.getHeight();
                int[] navcam_pixels = new int[width * height];

                //get the pixels out of bitmap
                navcam_bit_undistort.getPixels(navcam_pixels, 0, width, 0, 0, width, height);

                //get the luminance data
                navcam_luminance = new RGBLuminanceSource(width, height, navcam_pixels);

                //convert to binary image
                navcam_bin = new BinaryBitmap(new HybridBinarizer(navcam_luminance));

                //decoding QR result
                result = reader.decode(navcam_bin);
                qr_string = result.getText();

                if (qr_string != "") {

                    api.judgeSendDiscoveredQR(target_qr, qr_string);

                    switch(target_qr){
                        case 0: QRData.pos_x = removeIdentifier(qr_string); break;
                        case 1: QRData.pos_y = removeIdentifier(qr_string); break;
                        case 2: QRData.pos_z = removeIdentifier(qr_string); break;
                        case 3: QRData.qua_x= removeIdentifier(qr_string); break;
                        case 4: QRData.qua_y = removeIdentifier(qr_string); break;
                        case 5: QRData.qua_z = removeIdentifier(qr_string); break;
                    }

                    Log.i(TAG, "QR Found : " + qr_string);
                    return true;
                }

            } catch (Exception e) {
                Log.i(TAG, "QR Not Found : " + Integer.toString(retryTimes));
                qr_string = "";
            }

            try {
                Thread.sleep(100);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }

            retryTimes++;
        }

        return false;
    }

     */

    private Boolean decodeBitmap(int target_qr) {

        QRCodeReader reader = new QRCodeReader();
        //get camera data once to initialize width, height and pixels

        BinaryBitmap navcam_bin;
        com.google.zxing.Result result;

        int MAX_RETRY_TIMES = 30;
        int retryTimes = 0;

        String qr_string = "";
        while (qr_string == "" && retryTimes < MAX_RETRY_TIMES) {

            long start = System.currentTimeMillis();

            try{

                Bitmap image = api.getBitmapNavCam();

                int width = image.getWidth();
                int height = image.getHeight();
                int size = width * height;

                int[] pixels = new int[size];

                image.getPixels(pixels, 0, width, 0, 0, width, height);

                byte[] pixelsData = new byte[size];
                for (int i = 0; i < size; i++) {
                    pixelsData[i] = (byte) pixels[i];
                }

                PlanarYUVLuminanceSource source = new PlanarYUVLuminanceSource(
                        pixelsData, width, height, 0, 0, width,
                        height, false);

                //convert to binary image
                navcam_bin = new BinaryBitmap(new HybridBinarizer(source));

                //decoding QR result
                result = reader.decode(navcam_bin);
                qr_string = result.getText();

                if (qr_string != "") {

                    api.judgeSendDiscoveredQR(target_qr, qr_string);
                    Log.i(TAG, "QR Found : " + qr_string);

                    if (target_qr == 0){

                        QRData.storePosition(qr_string);

                    }else{

                        QRData.storeQuaternion(qr_string);
                    }

                    return true;
                }

                Kinematics kinematics = api.getTrustedRobotKinematics(5);;
                Kinematics.Confidence confidence = kinematics.getConfidence();

                if (confidence == Kinematics.Confidence.GOOD){

                    double curX = kinematics.getPosition().getX();
                    double curY = kinematics.getPosition().getY();
                    double curZ = kinematics.getPosition().getZ();

                    if (Math.abs(curX - 10.7) > 0.1 ||
                            Math.abs(curY + 5.16) > 0.1 ||
                            Math.abs(curZ - 4.42) > 0.1){

                        moveToWrapper(10.7, -5.16, 4.42, 0, 0 ,1, 0);

                    }
                }

            } catch (Exception e) {
                Log.i(TAG, "QR Not Found : " + retryTimes);
                qr_string = "";
            }

            retryTimes++;

            long end = System.currentTimeMillis();
            long elapsedTime = end - start;
            Log.d(TAG, "TIME SCANNING ELAPSED " + elapsedTime);
        }

        return false;
    }

    private Mat fastUndistort(Mat image){

        Mat map1 = new Mat();
        Mat map2 = new Mat();

        if(map1.empty() && map2.empty()){

            initUndistortRectifyMap(camMatrix, distCoeff, new Mat(), camMatrix, new Size(1280,960), CV_16SC2, map1, map2 );

        }

        Mat tmp = image.clone();
        remap(tmp, image, map1, map2, INTER_LINEAR, BORDER_CONSTANT);

        return image;
    }

    private void decodeWithZbar(int target_qr, Mat navcam_mat) {

        ImageScanner scanner;
        scanner = new ImageScanner();

        scanner.setConfig(0, Config.X_DENSITY, 3);
        scanner.setConfig(0, Config.Y_DENSITY, 3);

        scanner.setConfig(Symbol.NONE, Config.ENABLE, 0);
        // bar code
        scanner.setConfig(Symbol.I25, Config.ENABLE, 0);
        scanner.setConfig(Symbol.CODABAR, Config.ENABLE, 0);
        scanner.setConfig(Symbol.CODE128, Config.ENABLE, 0);
        scanner.setConfig(Symbol.CODE39, Config.ENABLE, 0);
        scanner.setConfig(Symbol.CODE93, Config.ENABLE, 0);
        scanner.setConfig(Symbol.DATABAR, Config.ENABLE, 0);
        scanner.setConfig(Symbol.DATABAR_EXP, Config.ENABLE, 0);
        scanner.setConfig(Symbol.EAN13, Config.ENABLE, 0);
        scanner.setConfig(Symbol.EAN8, Config.ENABLE, 0);
        scanner.setConfig(Symbol.ISBN10, Config.ENABLE, 0);
        scanner.setConfig(Symbol.ISBN13, Config.ENABLE, 0);
        scanner.setConfig(Symbol.UPCA, Config.ENABLE, 0);
        scanner.setConfig(Symbol.UPCE, Config.ENABLE, 0);
        scanner.setConfig(Symbol.PARTIAL, Config.ENABLE, 0);
        // qr code
        scanner.setConfig(Symbol.QRCODE, Config.ENABLE, 1);
        scanner.setConfig(Symbol.PDF417, Config.ENABLE, 1);

        int retry_times = 0;
        int MAX_RETRY_TIMES = 1;


        while(retry_times < MAX_RETRY_TIMES){

            //undistort

            Mat tmp = navcam_mat.clone();
            remap(tmp, navcam_mat, map1, map2, INTER_LINEAR, BORDER_CONSTANT);

            ImageSaver saver = new ImageSaver();
            saver.save_image(navcam_mat, "dfdf");

            //undistort

            Bitmap navcam_bitmap = matToBitmap(navcam_mat);

            int width = navcam_bitmap.getWidth();
            int height = navcam_bitmap.getHeight();
            int size = width * height;

            int[] pixels = new int[size];

            navcam_bitmap.getPixels(pixels, 0, width, 0, 0, width, height);

            byte[] pixelsData = new byte[size];
            for (int i = 0; i < size; i++) {
                pixelsData[i] = (byte) pixels[i];
            }

            net.sourceforge.zbar.Image barcode = new net.sourceforge.zbar.Image (width, height, "Y800");
            barcode.setData(pixelsData);

            int result = scanner.scanImage(barcode);

            if (result != 0) {
                SymbolSet syms = scanner.getResults();
                for (Symbol sym : syms) {
                    String resultStr = sym.getData();
                    api.judgeSendDiscoveredQR(target_qr, resultStr);
                    Log.i("QR FOUND: ", resultStr);

                }
            }else{
                Log.i("QR NOT FOUND", "QR NOT FOUND");
            }

            retry_times++;

            try {
                Thread.sleep(100);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        }

    }

}


