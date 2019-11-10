import java.util.*;

public class BLAST {

    final static String BLOSUMalpha = "ARNDCQEGHILKMFPSTWYV";

    private static float getScore(String A, String B) {
        if (A.length() == B.length()) {
            float score = 0;
            for (int i = 0; i < A.length(); i++)
                score += Blosum50.getScore(A.charAt(i), B.charAt(i));
            return score;
        } else {
            System.err.print("The strings must have the same size");
            return -1;
        }
    }

    private static String replaceCharAt(String s, int pos, char c) {
        return s.substring(0, pos) + c + s.substring(pos + 1);
    }

    private static ArrayList<TaggedString> getChildren(String original, int start, String child, float minScore, int index) {
        ArrayList<TaggedString> children = new ArrayList<>();

        for (int i = 0; i < 20; i++) {
            if (child.charAt(index) != BLOSUMalpha.charAt(i)) {
                String newChild = replaceCharAt(child, index, BLOSUMalpha.charAt(i));
                float newScore = getScore(newChild, original);
                if (newScore >= minScore) {
                    if (newChild.compareTo(original) != 0)
                        children.add(new TaggedString(newChild, start));
                    if (index + 1 < original.length())
                        children.addAll(getChildren(original, start, newChild, minScore, index + 1));
                }
            }
        }
        if (index + 1 < original.length())
            children.addAll(getChildren(original, start, child, minScore, index + 1));
        return children;
    }

    private static HashSet<TaggedString> generateSg(String[] Wg, float th) {
        HashSet<TaggedString> Sg = new HashSet<>();
        for (int i = 0; i < Wg.length; i++) {
            Sg.addAll(getChildren(Wg[i], i, Wg[i], getScore(Wg[i], Wg[i]) * th, 0));
            Sg.add(new TaggedString(Wg[i], i));
        }
        return Sg;
    }

    private static TaggedString[] createSg(String g, int k, float th) {
        String[] Wg = new String[g.length() - k + 1];
        for (int i = 0; i < Wg.length; i++)
            Wg[i] = g.substring(i, i + k);

        HashSet<TaggedString> SgHashSet = generateSg(Wg, th);
        return SgHashSet.toArray(new TaggedString[SgHashSet.size()]);
    }

    public static List<Integer> getMatchingIndices(String g, String t, int k, float th) {
        TaggedString[] Sg = createSg(g, k, th);

        AhoCorasick sm = new AhoCorasick();
        sm.createTrie(Sg);
        sm.getFailure();

        ArrayList<SearchResult> result = new ArrayList<>(sm.search(t, true));
        ArrayList<Integer> indices = new ArrayList<>();
        for (SearchResult r : result) {
            indices.add(r.getMatchPosition());
        }

        return indices;
    }

    public static ArrayList<String> getLocalAlignments(String g, String t, int k, float th, float thl, boolean showAlignments) {
        float gAutoScore = getScore(g, g);
        TaggedString[] Sg = createSg(g, k, th);

        AhoCorasick sm = new AhoCorasick();
        sm.createTrie(Sg);
        sm.getFailure();

        ArrayList<SearchResult> matches = new ArrayList<>(sm.search(t, true));
        HashSet<LocalAlignment> localAlignments = new HashSet<>();

        for (int i = 0; i < matches.size(); i++) {
            int start = matches.get(i).getMatchPosition();
            TaggedString taggedString = matches.get(i).getTaggedString();
            int tag = taggedString.getTag();

            int left = 0;
            while (start - left > 0 &&
                    tag - left > 0 &&
                    Blosum50.getScore(t.charAt(start - left - 1), g.charAt(tag - left - 1)) >= 0) {
                left++;
            }

            int right = 0;
            while (start + k + right < t.length() &&
                    tag + k + right < g.length() &&
                    Blosum50.getScore(t.charAt(start + k + right), g.charAt(tag + k + right)) >= 0) {
                right++;
            }

            String local = t.substring(start - left, start + k + right);
            String gLocal = g.substring(tag - left, tag) + taggedString.getString() + g.substring(tag + k, tag + k + right);

            float localScore = getScore(local, gLocal);
            if (localScore > gAutoScore * thl) {
                localAlignments.add(new LocalAlignment(start - left, k + left + right, localScore, gLocal));
            }
        }
        
        ArrayList<String> report = new ArrayList<>();
        if (showAlignments)
            report.add(t);
        for (LocalAlignment l : localAlignments) {
            
            String alignmentOut = "";
            if (showAlignments) {
                for (int j = 0; j < l.getStart(); j++) alignmentOut += " ";
                alignmentOut += l.getSequence();
                for (int j = 0; j <= t.length() - l.getStart() - l.getLength(); j++) alignmentOut += " ";
            } else {
                alignmentOut += l.getSequence() + " ";
            }

            alignmentOut += "(start = " + l.getStart() + ", length = " + l.getLength() + ") score = " + l.getScore();

            report.add(alignmentOut);

        }
        return report;
    }


}
