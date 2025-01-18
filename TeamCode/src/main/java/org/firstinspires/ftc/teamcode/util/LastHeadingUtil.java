//package org.firstinspires.ftc.teamcode.util;
//
//import java.io.BufferedReader;
//import java.io.FileReader;
//import java.io.FileWriter;
//import java.io.IOException;
//
//public class LastHeadingUtil {
//    private static double[] ret = new double[2];
//
//    public static double getLastHeading(){
//        readLastHeadingFromFile();
//        return ret[1];
//    }
//
//    public static void recordHeading(OpmodeType type, double recordThisAngle){
//        double[] newContents = new double[2];
//        if(type == OpmodeType.AUTO){
//            newContents[0] = 1;
//            newContents[0] = recordThisAngle;
//        }else{
//            newContents[0] = 0;
//            newContents[1] = 180;
//        }
//        writeLastHeadingToFile(newContents);
//    }
//
//    private static double[] readLastHeadingFromFile(){
//        String line = "";
//        double lastCount;
//        double lastHeading;
//
//        try {
//            BufferedReader br = new BufferedReader(new FileReader("lastheading.txt"));
//            while ((line = br.readLine()) != null) {
//                ret[0] = Double.parseDouble(line.split(",")[0]);
//                ret[1] = Double.parseDouble(line.split(",")[1]);
//            }
//        } catch (IOException e) {
//        }
//
//        return ret;
//    }
//
//    private static void writeLastHeadingToFile(double[] newContents){
//        try (FileWriter writer = new FileWriter("lastheading.txt")) {
//            writer.append(newContents[0]+","+newContents[1]);
//        } catch (IOException e) {
//            e.printStackTrace();
//        }
//    }
//
//    public enum OpmodeType{
//        TELEOP,
//        AUTO
//    }
//
//}
