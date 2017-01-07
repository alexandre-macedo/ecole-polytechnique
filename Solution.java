public class Solution {

	public static String LCS(String A, String B) {
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

	public static String OA(String A, String B) {
		int[][] F = new int[A.length() + 1][B.length() + 1];
		int[][] parent = new int[A.length() + 1][B.length() + 1];

		int d = -1;
		int b = 1;

		for (int i = 0; i <= A.length(); i++) {
			parent[i][0] = 1;
			F[i][0] = d * i;
		}
		for (int j = 0; j <= B.length(); j++) {
			parent[0][j] = 2;
			F[0][j] = d * j;
		}

		for (int i = 1; i <= A.length(); i++) {
			for (int j = 1; j <= B.length(); j++) {
				int match = F[i - 1][j - 1] + (A.charAt(i - 1) == B.charAt(j - 1) ? b : d);
				int delete = F[i - 1][j] + d;
				int insert = F[i][j - 1] + d;

				F[i][j] = match;
				if (delete > match && delete > insert) {
					parent[i][j] = 1;
					F[i][j] = delete;
				} else if (insert > match && insert > delete) {
					parent[i][j] = 2;
					F[i][j] = insert;
				}
			}
		}

		int matches = 0;
		int edits = 0;
		// Reconstruction of the solution
		String solutionA = "";
		String solutionB = "";
		int i = A.length(), j = B.length();
		while (i > 0 || j > 0) {
			if (i > 0 && j > 0 && parent[i][j] == 0) {
				i--;
				j--;
				if (A.charAt(i) == B.charAt(j)) {
					matches++;
					solutionA = "[" + A.charAt(i) + "]" + solutionA;
					solutionB = "[" + B.charAt(j) + "]" + solutionB;
				} else {
					edits++;
					solutionA = " " + A.charAt(i) + " " + solutionA;
					solutionB = " " + B.charAt(j) + " " + solutionB;
				}
			} else if (i > 0 && parent[i][j] == 1) {
				edits++;
				i--;
				solutionA = " " + A.charAt(i) + " " + solutionA;
				solutionB = " - " + solutionB;
			} else if (j > 0 && parent[i][j] == 2) {
				edits++;
				j--;
				solutionA = " - " + solutionA;
				solutionB = " " + B.charAt(j) + " " + solutionB;
			}
		}
		return solutionA + "\n" + solutionB + "\nMatches: " + matches + "\nEdits: " + edits;
	}
}
