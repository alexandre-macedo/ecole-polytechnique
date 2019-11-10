import java.lang.reflect.Array;
import java.util.ArrayList;

/**
 * Created by lugao on 1/9/17.
 */
public class OptmalAlignment {

    private static final int P = 0;
    private static final int Q = 1;
    private static final int D = 2;

    public static ArrayList<String> OptSimple(String A, String B) {
        int[][] F = new int[A.length() + 1][B.length() + 1];
        int[][] parent = new int[A.length() + 1][B.length() + 1];

        for (int i = 0; i <= A.length(); i++) {
            parent[i][0] = 1;
            F[i][0] = i;

        }
        for (int j = 0; j <= B.length(); j++) {
            parent[0][j] = 2;
            F[0][j] = j;
        }

        for (int i = 1; i <= A.length(); i++) {
            for (int j = 1; j <= B.length(); j++) {
                int min = F[i - 1][j - 1];
                if (A.charAt(i - 1) != B.charAt(j - 1)) {
                    min = F[i - 1][j - 1] + 1;
                    if (F[i - 1][j] + 1 <= min) {
                        parent[i][j] = 1;
                        min = F[i - 1][j] + 1;
                    }
                    if (F[i][j - 1] + 1 <= min) {
                        parent[i][j] = 2;
                        min = F[i][j - 1] + 1;
                    }
                }
                F[i][j] = min;
            }
        }

        return OptGlobalTraceback(A, B, parent, false);
    }

    public static ArrayList<String> OptGlobal(String A, String B) {
        float[][] F = new float[A.length() + 1][B.length() + 1];
        int[][] parent = new int[A.length() + 1][B.length() + 1];

        for (int i = 1; i <= A.length(); i++) {
            parent[i][0] = 1;
        }
        for (int j = 1; j <= B.length(); j++) {
            parent[0][j] = 2;
        }

        for (int i = 1; i <= A.length(); i++) {
            for (int j = 1; j <= B.length(); j++) {
                float match = F[i - 1][j - 1] + Blosum50.getScore(A.charAt(i - 1), B.charAt(j - 1));
                float insert = F[i - 1][j];
                float delete = F[i][j - 1];
                F[i][j] = match;
                if (F[i][j] < insert) {
                    F[i][j] = insert;
                    parent[i][j] = 1;
                }
                if (F[i][j] < delete) {
                    F[i][j] = delete;
                    parent[i][j] = 2;
                }
            }
        }

        ArrayList<String> alignment = OptGlobalTraceback(A, B, parent, true);
        alignment.add("Score: " + F[A.length()][B.length()]);

        return  alignment;
    }

    public static ArrayList<String> Opt(String A, String B, float gap, float ext) {
        int n = A.length();
        int m = B.length();

        float[][][] data = new float[n + 1][m + 1][3];
        int[][][] trace = new int[n + 1][m + 1][3];


        for (int i = 1; i <= n; i++)
            data[i][0][Q] = Float.NEGATIVE_INFINITY;
        for (int j = 1; j <= m; j++)
            data[0][j][P] = Float.NEGATIVE_INFINITY;

        data[0][0][D] = 0;

        for (int i = 1; i <= n; i++) {
            for (int j = 1; j <= m; j++) {
                data[i][j][P] = maximum(
                        data[i - 1][j][P] - ext,
                        data[i - 1][j][D] - gap,
                        trace[i][j], 0);

                data[i][j][Q] = maximum(
                        data[i][j - 1][Q] - ext,
                        data[i][j - 1][D] - gap,
                        trace[i][j], 1);

                data[i][j][D] = maximum(
                        data[i][j][P],
                        data[i][j][Q],
                        data[i - 1][j - 1][D] + Blosum50.getScore(A.charAt(i - 1), B.charAt(j - 1)),
                        trace[i][j], 2);
            }
        }
        int i = A.length(), j = B.length();
        float max = Float.NEGATIVE_INFINITY;

        for (int ii = 0; ii <= n; ii++) {
            if (data[ii][m][D] > max) {
                max = data[ii][m][D];
                i = ii;
                j = m;
            }
        }
        for (int jj = 0; jj <= m; jj++) {
            if (data[n][jj][D] > max) {
                max = data[n][jj][D];
                i = n;
                j = jj;
            }
        }
        ArrayList<String> alignment = OptTraceback(A, B, i, j, trace, gap, ext);
        alignment.add("Score = " + max);
        return alignment;
    }

    public static ArrayList<String> OptLoc(String A, String B, float gap, float ext) {
        int n = A.length();
        int m = B.length();

        int P = 0;
        int Q = 1;
        int D = 2;

        float[][][] data = new float[n + 1][m + 1][3];
        int[][][] trace = new int[n + 1][m + 1][3];

        for (int i = 1; i <= n; i++)
            data[i][0][Q] = Float.NEGATIVE_INFINITY;
        for (int j = 1; j <= m; j++)
            data[0][j][P] = Float.NEGATIVE_INFINITY;

        data[0][0][D] = 0;

        for (int i = 1; i <= n; i++) {
            for (int j = 1; j <= m; j++) {
                data[i][j][P] = maximum(data[i - 1][j][P] - ext,
                        data[i - 1][j][D] - gap,
                        trace[i][j], 0);

                data[i][j][Q] = maximum(data[i][j - 1][Q] - ext,
                        data[i][j - 1][D] - gap,
                        trace[i][j], 1);

                data[i][j][D] = maximum(data[i][j][P], data[i][j][Q],
                        data[i - 1][j - 1][D] + Blosum50.getScore(A.charAt(i - 1), B.charAt(j - 1)),
                        trace[i][j], 2);

                if (data[i][j][D] < 0)
                    data[i][j][D] = 0;
            }
        }

        int i = A.length(), j = B.length();
        float max = Float.NEGATIVE_INFINITY;

        for (int ii = 0; ii <= n; ii++)
            for (int jj = 0; jj <= m; jj++)
                if (data[ii][jj][D] > max) {
                    max = data[ii][jj][D];
                    i = ii;
                    j = jj;
                }
        ArrayList<String> alignment = OptLocTraceback(A, B, i, j, trace, data, gap, ext);
        alignment.add("Score = " + max);
        return alignment;
    }


    private static ArrayList<String> OptGlobalTraceback(String A, String B, int[][] parent, boolean blosum) {
        int matches = 0;
        int edits = 0;
        // Reconstruction of the solution
        String solutionA = "";
        String solutionB = "";
        String connection = "";
        int i = A.length(), j = B.length();
        while (i > 0 || j > 0) {
            if (i > 0 && j > 0 && parent[i][j] == 0) {
                i--;
                j--;
                if (A.charAt(i) == B.charAt(j)) {
                    matches++;
                    connection = "|" + connection;
                } else {
                    edits++;
                    if (blosum && Blosum50.getScore(A.charAt(i), B.charAt(j)) >= 0)
                        connection = ":" + connection;
                    else
                        connection = " " + connection;
                }
                solutionA = A.charAt(i) + solutionA;
                solutionB = B.charAt(j) + solutionB;
            } else if (i > 0 && parent[i][j] == 1) {
                edits++;
                i--;
                solutionA = A.charAt(i) + solutionA;

                solutionB = "-" + solutionB;
                connection = " " + connection;
            } else if (j > 0 && parent[i][j] == 2) {
                edits++;
                j--;
                solutionA = "-" + solutionA;
                solutionB = B.charAt(j) + solutionB;
                connection = " " + connection;
            }
        }
        ArrayList<String> alignment = new ArrayList<>();
        alignment.add(solutionA);
        alignment.add(connection);
        alignment.add(solutionB);
        return alignment;
    }

    private static ArrayList<String> OptTraceback(String A, String B, int i, int j, int[][][] trace, float gap, float ext) {
/*
        for (int ii = 0; ii <= A.length(); ii++) {
            for (int jj = 0; jj <= B.length(); jj++) {
                switch (trace[ii][jj][2]) {
                    case 0:
                        System.out.print("P");
                    break;
                    case 1:
                        System.out.print("Q");
                        break;
                    case 2:
                        System.out.print("↖");
                        break;
                }
                System.out.print("  ");
            }
            System.out.print("\n");
        }
        System.out.print("\n");
        for (int ii = 0; ii <= A.length(); ii++) {
            for (int jj = 0; jj <= B.length(); jj++) {
                switch (trace[ii][jj][0]) {
                    case 0:
                        System.out.print("↑");
                        break;
                    case 1:
                        System.out.print("D");
                        break;
                }
                System.out.print("  ");
            }
            System.out.print("\n");
        }
        System.out.print("\n");
        for (int ii = 0; ii <= A.length(); ii++) {
            for (int jj = 0; jj <= B.length(); jj++) {
                switch (trace[ii][jj][1]) {
                    case 0:
                        System.out.print("←");
                        break;
                    case 1:
                        System.out.print("D");
                        break;
                }
                System.out.print("  ");
            }
            System.out.print("\n");
        }*/
        String solutionA = "";
        String solutionB = "";
        String connection = "";

        int lastGapPos = (A.length() - i) > (B.length() - j) ? A.length() - i : B.length() - j;
        for (int ii = i; ii < A.length(); ii++) {
            solutionA += A.charAt(ii);
            solutionB += "-";
            connection += " ";
        }
        for (int jj = j; jj < B.length(); jj++) {
            solutionB += B.charAt(jj);
            solutionA += "-";
            connection += " ";
        }
        int counter = 1;
        int[][] table = new int[A.length() + 1][B.length() + 1];
        int ma = 2;
        while (i > 0 && j > 0) {
            table[i][j] = counter++;
            switch (ma) {
                case 0: // P
                    solutionA = A.charAt(i - 1) + solutionA;
                    solutionB = "-" + solutionB;
                    connection = " " + connection;
                    if (trace[i][j][ma] == 1) {
                        ma = 2;
                    }
                    i--;
                    break;
                case 1: // Q
                    solutionA = "-" + solutionA;
                    solutionB = B.charAt(j - 1) + solutionB;
                    connection = " " + connection;
                    if (trace[i][j][ma] == 1) {
                        ma = 2;
                    }
                    j--;
                    break;
                case 2: // D
                    switch (trace[i][j][ma]) {
                        case 0: // P .
                            ma = 0;
                            break;
                        case 1: // Q .
                            ma = 1;
                            break;
                        case 2: // D ↖
                            ma = 2;
                            solutionA = A.charAt(i - 1) + solutionA;
                            solutionB = B.charAt(j - 1) + solutionB;
                            if (A.charAt(i - 1) == B.charAt(j - 1))
                                connection = "|" + connection;
                            else if (Blosum50.getScore(A.charAt(i - 1), B.charAt(j - 1)) >= 0)
                                connection = ":" + connection;
                            else
                                connection = " " + connection;
                            i--;
                            j--;
                            break;
                    }
                    break;
            }
        }

        table[i][j] = counter++;
        /*System.out.print("\n");
        for (int ii = 0; ii <= A.length(); ii++) {
            for (int jj = 0; jj <= B.length(); jj++) {
                if(table[ii][jj] == 0)
                    System.out.print(".  ");
                else
                System.out.print(String.format("%-3d", table[ii][jj]));
            }
            System.out.print("\n");
        }*/

        for (int jj = j - 1; jj >= 0; jj--) {
            solutionB = B.charAt(jj) + solutionB;
            solutionA = "-" + solutionA;
            connection = " " + connection;
        }
        for (int ii = i - 1; ii >= 0; ii--) {
            solutionA = A.charAt(ii) + solutionA;
            solutionB = "-" + solutionB;
            connection = " " + connection;
        }

        ArrayList<String> alignment = new ArrayList<>();
        alignment.add(solutionA);
        alignment.add(connection);
        alignment.add(solutionB);
        alignment.add("Verified score = " + verifyScore(solutionA, solutionB, lastGapPos, ext, gap));
        return alignment;
    }

    private static ArrayList<String> OptLocTraceback(String A, String B, int i, int j, int[][][] trace, float[][][] data, float gap, float ext) {
        String solutionA = "";
        String solutionB = "";
        String connection = "";
        int ma = 2;
        while (i > 0 && j > 0 && data[i][j][ma] != 0) {
            switch (ma) {
                case 0: // P
                    solutionA = A.charAt(i - 1) + solutionA;
                    solutionB = "-" + solutionB;
                    connection = " " + connection;
                    if (trace[i][j][ma] == 1)
                        ma = 2;
                    i--;
                    break;
                case 1: // Q
                    solutionA = "-" + solutionA;
                    solutionB = B.charAt(j - 1) + solutionB;
                    connection = " " + connection;
                    if (trace[i][j][ma] == 1)
                        ma = 2;
                    j--;
                    break;
                case 2: // D
                    switch (trace[i][j][ma]) {
                        case 0: // P .
                            ma = 0;
                            break;
                        case 1: // Q .
                            ma = 1;
                            break;
                        case 2: // D ↖
                            ma = 2;
                            solutionA = A.charAt(i - 1) + solutionA;
                            solutionB = B.charAt(j - 1) + solutionB;
                            if (A.charAt(i - 1) == B.charAt(j - 1))
                                connection = "|" + connection;
                            else if (Blosum50.getScore(A.charAt(i - 1), B.charAt(j - 1)) >= 0)
                                connection = ":" + connection;
                            else
                                connection = " " + connection;
                            i--;
                            j--;
                            break;
                    }
                    break;
            }
        }

        ArrayList<String> alignment = new ArrayList<>();
        alignment.add(solutionA);
        alignment.add(connection);
        alignment.add(solutionB);
        alignment.add("Verified score = " + verifyScore(solutionA, solutionB, 0, ext, gap));
        return alignment;
    }


    private static float maximum(float arg0, float arg1, int[] trace, int id) {
        trace[id] = arg0 > arg1 ? 0 : 1;
        return arg0 > arg1 ? arg0 : arg1;
    }

    private static float maximum(float arg0, float arg1, float arg2, int[] trace, int id) {
        if (arg0 >= arg1 && arg0 >= arg2) {
            trace[id] = 0;
            return arg0;
        }
        if (arg1 >= arg0 && arg1 >= arg2) {
            trace[id] = 1;
            return arg1;
        }
        trace[id] = 2;
        return arg2;
    }

    private static float verifyScore(String solutionA, String solutionB, int gapShift, float ext, float gap) {
//Score verification
        float score = 0;
        boolean firstMatch = false;
        boolean inGap = false;
        String scoreStr = "";
        for (int x = 0; x < solutionA.length() - gapShift; x++) {
            if ((solutionA.charAt(x) != '-' && solutionB.charAt(x) != '-') || firstMatch) {
                firstMatch = true;
                if (solutionA.charAt(x) == '-' || solutionB.charAt(x) == '-') {
                    if (inGap) {
                        score -= ext;
                        scoreStr += ".  ";
                    } else {
                        score -= gap;
                        inGap = true;
                        scoreStr += "^  ";
                    }
                } else {
                    inGap = false;
                    score += Blosum50.getScore(solutionA.charAt(x), solutionB.charAt(x));
                    scoreStr += String.format("%-3d", (int) (Blosum50.getScore(solutionA.charAt(x), solutionB.charAt(x))));
                }
            } else
                scoreStr += "   ";
        }
        return score;
        /*String space = "  ";
        String nSolA = "";
        String nSolB = "";
        String nConn = "";
        for (int x = 0; x < solutionA.length(); x++) {
            nSolA += solutionA.charAt(x) + space;
            nSolB += solutionB.charAt(x) + space;
            nConn += connection.charAt(x) + space;
        }
        solutionA = nSolA;
        solutionB = nSolB;
        connection = nConn;
        //scoreStr = "";*/
    }
}
