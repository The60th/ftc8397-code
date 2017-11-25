package org.firstinspires.ftc.teamcode.beta_log;


import com.qualcomm.robotcore.util.ElapsedTime;

import java.io.BufferedWriter;
import java.io.FileWriter;
import java.util.Calendar;
import java.util.GregorianCalendar;

/**
 * Created by JimLori on 11/18/2017.
 */

public class BetaLog {

    private static BufferedWriter bufferedWriter = null;
    private static final String defaultPath = "/sdcard/BetaLog.txt";
    private static ElapsedTime elapsedTime = null;

    public static boolean initialize(){
        if (bufferedWriter != null) return true;
        elapsedTime = new ElapsedTime();
        GregorianCalendar gregorianCalendar = new GregorianCalendar();
        String header = String.format("BETA_LOG_INITIALIZED: %d:%d:%d %d:%d:%d:%d",
                gregorianCalendar.get(Calendar.YEAR), gregorianCalendar.get(Calendar.MONTH),
                gregorianCalendar.get(Calendar.DAY_OF_MONTH), gregorianCalendar.get(Calendar.HOUR_OF_DAY),
                gregorianCalendar.get(Calendar.MINUTE), gregorianCalendar.get(Calendar.SECOND),
                gregorianCalendar.get(Calendar.MILLISECOND));
        try {
            bufferedWriter = new BufferedWriter(new FileWriter(defaultPath, true));
            bufferedWriter.newLine();
            bufferedWriter.newLine();
            bufferedWriter.newLine();
            writeLine(header);
        }
        catch(java.io.IOException e){
            return false;
        }
        return true;
    }

    public static void close(){
        if (bufferedWriter != null){
            try{
                bufferedWriter.close();
            }
            catch (java.io.IOException e){
                return;
            }
            return;
        }
    }

    private static void writeLine(String string){
        if (bufferedWriter == null) return;
        try{
            bufferedWriter.write(string, 0, string.length());
            bufferedWriter.newLine();
        }
        catch (java.io.IOException e){
            return;
        }
        return;
    }


    public static void d(String format, Object... args) { d(String.format(format, args)); }
    public static void d(String message) { internalLog(message); }

    public static void dd(String tag, String format, Object... args) { dd(tag, String.format(format, args)); }
    public static void dd(String tag, String message) {
        internalLog(tag + ": " + message);
    }

    public static void internalLog( String message ){
        writeLine(String.format("  %.4f %s", elapsedTime.seconds(), message));
    }



}
