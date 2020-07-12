package jp.jaxa.iss.kibo.rpc.indonesia;

import android.graphics.Bitmap;
import android.os.Environment;
import android.util.Log;
import org.opencv.android.Utils;
import org.opencv.core.CvException;
import org.opencv.core.Mat;
import java.io.File;
import java.io.FileOutputStream;
import java.io.IOException;

import static android.content.ContentValues.TAG;

public class ImageSaver {

    public static void save_image(Mat subimg){

        Bitmap bmp = null;
        try {
            bmp = Bitmap.createBitmap(subimg.cols(), subimg.rows(), Bitmap.Config.ARGB_8888);
            Utils.matToBitmap(subimg, bmp);
        } catch (CvException e) {
            Log.d(TAG, e.getMessage());
        }

        //subimg.release();


        FileOutputStream out = null;

        String filename = "frame.png";


        File sd = new File(Environment.getExternalStorageDirectory() + "/frames");
        boolean success = true;
        if (!sd.exists()) {
            success = sd.mkdir();
        }
        if (success) {
            File dest = new File(sd, filename);

            try {
                out = new FileOutputStream(dest);
                bmp.compress(Bitmap.CompressFormat.PNG, 100, out); // bmp is your Bitmap instance
                // PNG is a lossless format, the compression factor (100) is ignored

            } catch (Exception e) {
                e.printStackTrace();
                Log.d(TAG, e.getMessage());
            } finally {
                try {
                    if (out != null) {
                        out.close();
                        Log.d(TAG, "OK!!");
                    }
                } catch (IOException e) {
                    Log.d(TAG, e.getMessage() + "Error");
                    e.printStackTrace();
                }
            }
        }

    }
}