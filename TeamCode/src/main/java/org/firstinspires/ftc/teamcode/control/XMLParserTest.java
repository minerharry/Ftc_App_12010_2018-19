package org.firstinspires.ftc.teamcode.control;

import android.content.res.Resources;
import android.content.res.XmlResourceParser;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.R;
import org.xmlpull.v1.XmlPullParserException;

import java.io.IOException;
@TeleOp(name = "xmlParseTest")
public class XMLParserTest extends OpMode {

    private XmlResourceParser parser = null;

    public synchronized void waitForStart()
    {
        resetStartTime();
    }

    @Override
    public void init() {

    }

    public void loop()
    {
        telemetry.addData("MotorName", hardwareMap.appContext.getResources().getString(R.string.backLeft));
        parser = hardwareMap.appContext.getResources().getXml(R.xml.teamwebcamcalibrations);
        int eventType = -1;
        // Loop through the XML data
        while (eventType != parser.END_DOCUMENT) {
            if (eventType == XmlResourceParser.START_TAG) {
                String nameValue = parser.getName();
                telemetry.addData("current Tag Name:",
                        nameValue);
                if (nameValue.equals("string")) {
                    String realValue = parser.getText();
                    telemetry.addData(nameValue, realValue);
                }
                try {
                    wait(2000);
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }
            }
            /*
                The method next() advances the parser to the next event. The int value returned from
                next determines the current parser state and is identical to the value returned
                from following calls to getEventType ().

                The following event types are seen by next()

                    START_TAG
                        An XML start tag was read.
                    TEXT
                        Text content was read; the text content can be retrieved using the getText()
                        method. (when in validating mode next() will not report ignorable
                        whitespace, use nextToken() instead)
                    END_TAG
                        An end tag was read
                    END_DOCUMENT
                        No more events are available
            */

            try {
                eventType = parser.next();
            } catch (XmlPullParserException e) {
                e.printStackTrace();
            } catch (IOException e) {
                e.printStackTrace();
            }
        }
        //telemetry.addData("MotorName2", Resources.getSystem().getString(R.string.backLeft));
    }

    public void runOpMode() {
        waitForStart();
        int motorThing = hardwareMap.appContext.getResources().getInteger(R.integer.backward_jewel_angle_percent);
        telemetry.addData("Back Left", motorThing);

        double startTime = getRuntime();
        while(getRuntime() < startTime + 10)
        {
            telemetry.addData("Back Left", motorThing);
            telemetry.addData("DDDDd", "ssssss");
           // idle();
        }
        parser = hardwareMap.appContext.getResources().getXml(R.xml.teamwebcamcalibrations);
        int eventType = -1;

        // Loop through the XML data
        while (eventType != parser.END_DOCUMENT) {
            if (eventType == XmlResourceParser.START_TAG) {
                String nameValue = parser.getName();
                if (nameValue.equals("string")) {
                    String realValue = parser.getText();
                    telemetry.addData(nameValue, realValue);
                }
            }
            /*
                The method next() advances the parser to the next event. The int value returned from
                next determines the current parser state and is identical to the value returned
                from following calls to getEventType ().

                The following event types are seen by next()

                    START_TAG
                        An XML start tag was read.
                    TEXT
                        Text content was read; the text content can be retrieved using the getText()
                        method. (when in validating mode next() will not report ignorable
                        whitespace, use nextToken() instead)
                    END_TAG
                        An end tag was read
                    END_DOCUMENT
                        No more events are available
            */

            try {
                eventType = parser.next();
            } catch (XmlPullParserException e) {
                e.printStackTrace();
            } catch (IOException e) {
                e.printStackTrace();
            }
        }
    }
}
