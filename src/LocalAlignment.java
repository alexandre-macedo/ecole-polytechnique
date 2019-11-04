public class LocalAlignment {
    private final int start;
    private final int length;
    private final float score;
    private final String sequence;

    public LocalAlignment(int start, int length, float score, String sequence) {
        this.start = start;
        this.length = length;
        this.score = score;
        this.sequence = sequence;
    }

    public int getStart() {
        return start;
    }

    public int getLength() {
        return length;
    }

    public float getScore() {
        return score;
    }

    public String getSequence() {
        return sequence;
    }

    public int hashCode() {
        return Integer.hashCode(start) + Integer.hashCode(length) + Float.hashCode(score) + sequence.hashCode();
    }

    public boolean equals(Object obj) {
        if (obj.getClass() != this.getClass())
            return false;
        if (((LocalAlignment) obj).getStart() == start &&
                ((LocalAlignment) obj).getLength() == length &&
                ((LocalAlignment) obj).getScore() == score &&
                ((LocalAlignment) obj).getSequence().compareTo(sequence) == 0)
            return true;
        return false;
    }
}