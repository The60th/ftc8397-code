package org.firstinspires.ftc.teamcode.beta_log;

import com.qualcomm.robotcore.util.ElapsedTime;

import java.io.BufferedWriter;
import java.io.FileWriter;
import java.util.Calendar;
import java.util.GregorianCalendar;

/**
 * Created by JimLori on 11/20/2017.
 * Modifications on 11/25/2017 to prevent exceptions when methods are called without initialization.
 * In addition, initialize() method modified so that if an exception occurs,
 * the bufferedWriter gets closed then set to null. The close() method is modified so that it sets bufferedWriter
 * to null before returning. With these changes, calls to the public methods of BetaLog (other than
 * initialize() and close() ) will be "do-nothing" statements if BetaLog has not been initialized,
 * if initialization has failed, or if it has already been closed.
 */

public class BetaLog {

    private static BufferedWriter bufferedWriter = null;
    private static final String defaultPath = "/sdcard/BetaLog.txt";
    private static ElapsedTime elapsedTime = null;

    //Instantiate bufferedWriter and write the header. If this fails, close the bufferedWriter, then
    //set it to null.
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
            close();
            return false;
        }
        return true;
    }

    //Close the bufferedWriter, then set it to null
    public static void close(){
        if (bufferedWriter != null){
            try{
                bufferedWriter.close();
            }
            catch (java.io.IOException e){
                return;
            }
            finally {
                bufferedWriter = null;
            }
        }
    }

    //Write a string to the log file, followed by a new line
    private static void writeLine(String string){
        if (bufferedWriter == null) return;
        try{
            bufferedWriter.write(string, 0, string.length());
            bufferedWriter.newLine();
        }
        catch (java.io.IOException e){
            return;
        }
    }


    //Public logging methods, all of which are ultimately dependent on internalLog method:

    //Without TAG
    public static void d(String format, Object... args) { d(String.format(format, args)); }
    public static void d(String message) { internalLog(message); }

    //With TAG
    public static void dd(String tag, String format, Object... args) { dd(tag, String.format(format, args)); }
    public static void dd(String tag, String message) {
        internalLog(tag + ": " + message);
    }

    //Write message to the log file, preceeded by elapsed time (since initialization) in seconds
    private static void internalLog( String message ){
        if (bufferedWriter == null) return;
        String string = String.format("  %.4f %s", elapsedTime.seconds(), message);
        try{
            bufferedWriter.write(string, 0, string.length());
            bufferedWriter.newLine();
        }
        catch (java.io.IOException e){
            return;
        }
    }

}
