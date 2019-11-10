/**
 * Created by lugao on 1/9/17.
 */
public class LCS {
    public static String Calculate(String A, String B) {
        int[][] LCS = new int[A.length() + 1][B.length() + 1];
        int[][] parent = new int[A.length() + 1][B.length() + 1];

        // Dynamic programming solution
        final int MATCH = 0, LEFT = 1, RIGHT = 2;
        for (int i = 1; i <= A.length(); i++)
            for (int j = 1; j <= B.length(); j++)
                if (A.charAt(i - 1) == B.charAt(j - 1)) {
                    LCS[i][j] = LCS[i - 1][j - 1] + 1;
                    parent[i][j] = MATCH;
                } else {
                    LCS[i][j] = Math.max(LCS[i - 1][j], LCS[i][j - 1]);
                    parent[i][j] = LCS[i - 1][j] < LCS[i][j - 1] ? RIGHT : LEFT;
                }

        // Reconstruction of the solution
        String solution = "";
        int i = A.length(), j = B.length();
        while (i != 0 && j != 0)
            if (parent[i][j] == MATCH) {
                i--;
                j--;
                solution = A.charAt(i) + solution;
            } else if (parent[i][j] == LEFT)
                i--;
            else
                j--;

        return solution;
    }
}
