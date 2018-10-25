package org.firstinspires.ftc.teamcode.control;

import android.content.res.Resources;
import android.content.res.XmlResourceParser;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.R;
import org.xmlpull.v1.XmlPullParserException;

import java.io.IOException;
@TeleOp(name = "xmlParseTest")
public class XMLParserTest extends OpMode {

    private XmlResourceParser parser = null;
    @Override
    public void init()
    {
        parser = Resources.getSystem().getXml(R.xml.atest);
        int eventType = -1;

        // Loop through the XML data
        while(eventType!=parser.END_DOCUMENT){
            if(eventType == XmlResourceParser.START_TAG){
                String nameValue = parser.getName();
                if (nameValue.equals("student")){
                    String realValue = parser.getText();
                    telemetry.addData(nameValue,realValue);
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
    @Override
    public void loop()
    {

    }
}
