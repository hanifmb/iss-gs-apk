package jp.jaxa.iss.kibo.rpc.indonesia;

import java.io.File;
import java.util.ArrayList;
import java.util.List;

public class imageFileUtils {

    static File fileLocation = null;

    public static List<File> iterateOverFiles(File[] files) {
        List<File> result = new ArrayList<>();
        for (File file : files) {
            if (file.isDirectory()) {
                result.addAll(iterateOverFiles(file.listFiles())); // Here!
            } else {
                    fileLocation = findFileswithJpgExtension(file);
                if(fileLocation != null) {
                    result.add(fileLocation);
                }
            }
        }

        return result;
    }

    public static File findFileswithJpgExtension(File file) {

        if(file.getName().toLowerCase().endsWith("jpg")) {
            return file;
        }

        return null;
    }

}
