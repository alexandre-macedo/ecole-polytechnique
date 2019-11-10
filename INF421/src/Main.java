import java.nio.file.Files;
import java.nio.file.Paths;
import java.util.ArrayList;
import java.util.List;

public class Main {
    public static void printLines(ArrayList<String> lines) {
        for (String line : lines)
            System.out.println(line);
    }

    public static void main(String[] args) {
        List<String> file = new ArrayList<>();
        try {
            file = Files.readAllLines(Paths.get("teste.txt"));
        } catch (Exception e) {
            System.err.println(e.toString());
        }

        if (file.size() >= 4) {
            System.out.println("Task2 (Longest common subsequences):\n");
            System.out.println(LCS.Calculate(file.get(0), file.get(1)));

            System.out.println("\nTask3 (Optimal alignment):\n");
            printLines(OptmalAlignment.OptSimple(file.get(0), file.get(1)));
            System.out.println("\nTask4 (Optimal alignment with BLOSUM50):\n");
            printLines(OptmalAlignment.OptGlobal(file.get(0), file.get(1)));

            System.out.println("\nTask5 (Semi-global alignment with affine penalty):\n");
            printLines(OptmalAlignment.Opt(file.get(0), file.get(1), 10, 1));
            System.out.println("\nTask6 (Local alignment with affine penalty):\n");
            printLines(OptmalAlignment.OptLoc(file.get(0), file.get(1), 10, 1));

            int k = 4;
            float th = 0.8F;
            float thl = 0.1F;

            System.out.println("\nTask7 (BLAST):\n");
            System.out.println(BLAST.getMatchingIndices(file.get(2), file.get(3), k, th));
            System.out.println("\nTask7 (BLAST local alignment):\n");
            printLines(BLAST.getLocalAlignments(file.get(2), file.get(3), k, th, thl, true));
        }

    }
}
