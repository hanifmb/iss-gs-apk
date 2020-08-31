package jp.jaxa.iss.kibo.rpc.indonesia;

public class QRData {

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
