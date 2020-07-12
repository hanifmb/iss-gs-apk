package jp.jaxa.iss.kibo.rpc.indonesia;

import jp.jaxa.iss.kibo.rpc.api.KiboRpcService;

import android.graphics.Bitmap;
import android.os.Environment;
import android.util.Log;

import gov.nasa.arc.astrobee.Result;
import gov.nasa.arc.astrobee.types.Point;
import gov.nasa.arc.astrobee.types.Quaternion;

import com.google.zxing.BinaryBitmap;
import com.google.zxing.RGBLuminanceSource;
import com.google.zxing.common.HybridBinarizer;
import com.google.zxing.qrcode.QRCodeReader;

import org.opencv.android.Utils;
import org.opencv.aruco.Aruco;
import org.opencv.aruco.DetectorParameters;
import org.opencv.aruco.Dictionary;
import org.opencv.core.CvException;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import static org.opencv.imgcodecs.Imgcodecs.imread;

import java.util.ArrayList;
import java.util.List;




/**
 * Class meant to handle commands from the Ground Data System and execute them in Astrobee
 */


public class YourService extends KiboRpcService {

    private Mat camMatrix;
    private Mat distCoeff;

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

        //moveToWrapper(11, -5.30, 4.33, 0.500, 0.500, -0.500, 0.500);
        //moveToWrapper(11, -5.60, 4.33, 0.500, 0.500, -0.500, 0.500);

        moveToWrapper(10.7, -5.16, 4.42, 0,0 , 1, 0);
        long start, end;

        for (int i = 0; i<2; i++){

            start = System.currentTimeMillis();
            decodeQRWithCam(0, Camera.NAVCAM, Image.DISTORT, 640, 480, "640x480_DISTORT");
            end = System.currentTimeMillis();
            Log.i("SPACECAT", "640x480_DISTORT" + String.valueOf(start-end));

            start = System.currentTimeMillis();
            decodeQRWithCam(0, Camera.NAVCAM, Image.UNDISTORT, 640, 480, "640x480_UNDISTORT");
            end = System.currentTimeMillis();
            Log.i("SPACECAT", "640x480_UNDISTORT" + String.valueOf(start-end));

            start = System.currentTimeMillis();
            decodeQRWithCam(0, Camera.NAVCAM, Image.DISTORT, 960, 720, "960x720_DISTORT");
            end = System.currentTimeMillis();
            Log.i("SPACECAT", "960x720_DISTORT" + String.valueOf(start-end));

            start = System.currentTimeMillis();
            decodeQRWithCam(0, Camera.NAVCAM, Image.UNDISTORT, 960, 720, "960x720_UNDISTORT");
            end = System.currentTimeMillis();
            Log.i("SPACECAT", "960x720_UNDISTORT" + String.valueOf(start-end));

            start = System.currentTimeMillis();
            decodeQRWithCam(0, Camera.NAVCAM, Image.DISTORT, 1280, 960, "1280x960_DISTORT");
            end = System.currentTimeMillis();
            Log.i("SPACECAT", "1280x960_DISTORT" + String.valueOf(start-end));

            start = System.currentTimeMillis();
            decodeQRWithCam(0, Camera.NAVCAM, Image.UNDISTORT, 1280, 960, "1280x960_UNDISTORT");
            end = System.currentTimeMillis();
            Log.i("SPACECAT", "1280x960_UNDISTORT" + String.valueOf(start-end));

        }

        //api.judgeSendFinishISS();
        api.judgeSendFinishSimulation();
    }


    @Override
    protected void runPlan2() {

        scanSDImages();

    }

    @Override
    protected void runPlan3() {

    }

    private Boolean moveToWrapper(double pos_x, double pos_y, double pos_z,
                                  double qua_x, double qua_y, double qua_z,
                                  double qua_w) {

        final int LOOP_MAX = 2000;
        final Point point = new Point(pos_x, pos_y, pos_z);
        final Quaternion quaternion = new Quaternion((float) qua_x, (float) qua_y,
                (float) qua_z, (float) qua_w);

        Result result = api.moveTo(point, quaternion, true);

        int loopCounter = 1;
        while (!result.hasSucceeded() && loopCounter <= LOOP_MAX) {

            result = api.moveTo(point, quaternion, true);
            ++loopCounter;

        }

        Boolean arrived = result.hasSucceeded() ? true : false;
        return arrived;
    }

    private boolean decodeQRWithCam(int targetQR, Camera camera, Image image, int resizeWidth, int resizeHeight, String identifier){

        Mat inputImage;

        int MAX_RETRY_TIMES = 1;
        int retryTimes = 1;

        String qrString = null;
        while (qrString == null && retryTimes <= MAX_RETRY_TIMES) {

            switch (camera) {
                case DOCKCAM:
                    inputImage = api.getMatDockCam();
                    break;
                default:
                    inputImage = api.getMatNavCam();
                    break;
            }

            boolean success = decodeQR(targetQR, inputImage, resizeWidth, resizeHeight, image, identifier);

            retryTimes++;

        }

        return false;
    }

    private Boolean decodeQR(int targetQR, Mat inputImage, int resizeWidth, int resizeHeight,  Image image, String identifier) {

        String qrString = null;

        QRCodeReader reader = new QRCodeReader();

        Mat imageResizedMat = new Mat();
        Bitmap imageResizedBitmap;
        BinaryBitmap navcamBin;

        Size size = new Size(resizeWidth, resizeHeight);

        RGBLuminanceSource navcamLuminance;
        com.google.zxing.Result result;

        reader.reset();

        if (image == Image.UNDISTORT){

            inputImage = undistortCamera(inputImage, camMatrix, distCoeff);

        }

        Imgproc.resize(inputImage, imageResizedMat, size);

        ImageSaver saver = new ImageSaver();
        saver.save_image(imageResizedMat);

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

            result = reader.decode(navcamBin);
            qrString = result.getText();

        } catch (Exception e) {

            Log.i("SPACECAT", identifier + "QR NOT FOUND");
            qrString = null;

        }

        if (qrString != null) {

            Log.i("SPACECAT", identifier+ qrString);

            //api.judgeSendDiscoveredQR(0, qrString);
            return true;

        }

        return false;
    }

    private void scanSDImages(){

        Mat img = imread(Environment.getExternalStorageDirectory().getAbsolutePath() +"/hanif/" + "qrlandscape.png");

    }


    private String removeIdentifier(String rec_data){
        String[] arrOfStr;

        //split the data to acquire only the component's value
        arrOfStr = rec_data.split(",", 2);

        //remove all white spaces
        arrOfStr[1].replaceAll("\\s+","");

        return arrOfStr[1];
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
                QRData.ar_id = Integer.toString(ARint);

                Log.i("AR FOUND: ", QRData.ar_id);
                api.judgeSendDiscoveredAR(QRData.ar_id);

                //Pose estimation
                Aruco.estimatePoseSingleMarkers(corners, 0.05f, camMatrix, distCoeff, rvec, tvec);

                /*
                Log.i("Corners", Integer.toString(corners.size()));
                Log.i("id", markerIds.dump());
                Log.i("rejected", Integer.toString(rejected.size()));
                */

                return tvec;
            }

            Log.i("AR NOT FOUND", "TRIAL " + (i + 1));

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

//Struct like container for QR position and quaternion string data
class QRData {

    static String pos_x;
    static String pos_y;
    static String pos_z;
    static String qua_x;
    static String qua_y;
    static String qua_z;
    static String ar_id;

}