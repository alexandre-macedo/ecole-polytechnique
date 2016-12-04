
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
		int[][] LCS = new int[A.length() + 1][B.length() + 1];
		int[][] parent = new int[A.length() + 1][B.length() + 1];

		// Dynamic programming solution
		final int MATCH = 0, LEFT = 1, RIGHT = 2, JUMP = 3;
		for (int i = 1; i <= A.length(); i++)
			for (int j = 1; j <= B.length(); j++)
				if (A.charAt(i - 1) == B.charAt(j - 1)) {
					LCS[i][j] = LCS[i - 1][j - 1] + 1;
					parent[i][j] = MATCH;
				} else {
					int dir = JUMP;
					// if (LCS[i][j - 1] != LCS[i - 1][j])
					dir = LCS[i][j - 1] > LCS[i - 1][j] ? RIGHT : LEFT;
					switch (dir) {
					case JUMP:
						LCS[i][j] = LCS[i - 1][j - 1];
						break;
					case LEFT:
						LCS[i][j] = LCS[i - 1][j];
						break;
					case RIGHT:
						LCS[i][j] = LCS[i][j - 1];
						break;
					}
					parent[i][j] = dir;
				}

		// Reconstruction of the solution
		String solutionA = "";
		String solutionB = "";
		int i = A.length(), j = B.length();
		while (i != 0 && j != 0) {
			System.out.println(parent[i][j]);
			if (parent[i][j] == MATCH || parent[i][j] == JUMP) {
				i--;
				j--;
				solutionA = A.charAt(i) + solutionA;
				solutionB = B.charAt(j) + solutionB;
			} else if (parent[i][j] == LEFT) {
				i--;
				solutionA = A.charAt(i) + solutionA;
				solutionB = "-" + solutionB;
			} else {
				j--;
				solutionA = "-" + solutionA;
				solutionB = B.charAt(j) + solutionB;
			}
		}
		while (i != 0) {
			i--;
			solutionA = A.charAt(i) + solutionA;
			solutionB = "-" + solutionB;
		}
		while (j != 0) {
			j--;
			solutionA = "-" + solutionA;
			solutionB = B.charAt(j) + solutionB;
		}
		return solutionA + "\n" + solutionB;
	}
}
