package trajectorysequence;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import trajectorysequence.sequencesegment.SequenceSegment;

import java.util.Collections;
import java.util.List;

public class TrajectorySequence {
    public final List<SequenceSegment> sequenceList;

    public TrajectorySequence(List<SequenceSegment> sequenceList) {
        this.sequenceList = Collections.unmodifiableList(sequenceList);
    }

    public Pose2d start() {
        return sequenceList.get(0).getStartPose();
    }

    public Pose2d end() {
        return sequenceList.get(sequenceList.size() - 1).getEndPose();
    }

    public double duration() {
        double total = 0.0;

        for (SequenceSegment segment : sequenceList) {
            total += segment.getDuration();
        }

        return total;
    }

    public SequenceSegment get(int i) {
        if (sequenceList.isEmpty()) return null;
        return sequenceList.get(i);
    }

    public int size() {
        if (sequenceList.isEmpty()) return -1;
        return sequenceList.size();
    }
}
