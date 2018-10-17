package org.firstinspires.ftc.teamcode.vision.dogeCVmodifications;

import android.util.Log;

import com.disnodeteam.dogecv.DogeCV;
import com.disnodeteam.dogecv.detectors.DogeCVDetector;
import com.disnodeteam.dogecv.detectors.roverrukus.SilverDetector;
import com.disnodeteam.dogecv.filters.DogeCVColorFilter;
import com.disnodeteam.dogecv.filters.HSVRangeFilter;
import com.disnodeteam.dogecv.scoring.MaxAreaScorer;
import com.disnodeteam.dogecv.scoring.PerfectAreaScorer;
import com.disnodeteam.dogecv.scoring.RatioScorer;

import org.firstinspires.ftc.teamcode.vision.SilverExampleModified;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;
import java.util.List;

/**
 * Created by Victo on 9/10/2018.
 */

public class SilverDetectorModified extends SilverDetector {
    public RoundnessScorer roundnessScorer = new RoundnessScorer(2);

    @Override
    public void useDefaults()
    {
        super.useDefaults();
        SilverExampleModified.display("Starting add scorer");
        addScorer(roundnessScorer);
        SilverExampleModified.display("Completed add scorer");
    }
}
