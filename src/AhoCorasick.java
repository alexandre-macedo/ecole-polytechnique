import java.util.*;

public class AhoCorasick {
    protected Map<Integer, state> stateMap;
    protected Map<Integer, Integer> Failure;
    protected Map<Integer, LinkedList<TaggedString>> Output;
    protected int numStates;
    protected state zeroState;

    protected class state {
        private int id;
        private char val;
        Map<Character, Integer> nextStates = new HashMap<>();

        public state(int id, char val) {
            this.id = id;
            this.val = val;
        }

        public int getId() {
            return this.id;
        }

        public char getVal() {
            return this.val;
        }
    }

    public AhoCorasick() {
        stateMap = new HashMap<>();
        Failure = new HashMap<>();
        Output = new HashMap<>();
        numStates = 0;
    }

    protected void outputAdd(int id, TaggedString str) {
        LinkedList<TaggedString> l;
        if (Output.containsKey(id)) {
            l = Output.get(id);
        } else {
            l = new LinkedList<TaggedString>();
            Output.put(id, l);
        }
        l.add(str);

    }

    protected void outputAdd(int id, LinkedList<TaggedString> list) {
        if (list == null || list.size() == 0) {
            return;
        }
        LinkedList<TaggedString> l;
        if (Output.containsKey(id)) {
            l = Output.get(id);
        } else {
            l = new LinkedList<TaggedString>();
            Output.put(id, l);
        }
        Iterator it = list.iterator();
        while (it.hasNext()) {
            l.add((TaggedString) it.next());
        }
    }

    protected void createTrie(TaggedString[] input) {
        zeroState = new state(0, '\0');
        stateMap.put(0, zeroState);
        state currState;
        for (int i = 0; i < input.length; i++) {
            currState = zeroState;
            int j = 0;
            while (true) {
                if (j >= input[i].getString().length()) {
                    break;
                }
                if (currState.nextStates.containsKey(input[i].getString().charAt(j))) {
                    currState = stateMap.get(currState.nextStates.get(input[i].getString().charAt(j)));
                    j++;
                } else {
                    break;
                }
            }
            while (j < input[i].getString().length()) {
                state nextState = new state(++numStates, input[i].getString().charAt(j));
                stateMap.put(numStates, nextState);
                currState.nextStates.put(input[i].getString().charAt(j), numStates);
                currState = nextState;
                j++;
            }
            outputAdd(currState.getId(), input[i]);
        }
    }

    protected void getFailure() {
        state currState = zeroState;
        LinkedList<Integer> q = new LinkedList<Integer>();
        Iterator it = currState.nextStates.entrySet().iterator();
        while (it.hasNext()) {
            Map.Entry pairs = (Map.Entry) it.next();
            int cs = (Integer) pairs.getValue();
            Failure.put(cs, 0);
            q.add(cs);
        }
        while (q.size() > 0) {
            int cs = q.removeFirst();
            it = stateMap.get(cs).nextStates.entrySet().iterator();
            while (it.hasNext()) {
                Map.Entry pairs = (Map.Entry) it.next();
                int ns = (Integer) pairs.getValue();
                char nk = (Character) pairs.getKey();
                q.add(ns);
                int fs = Failure.get(cs);
                while (true) {
                    if (fs == 0 || stateMap.get(fs).nextStates.containsKey(nk))
                        break;
                    fs = Failure.get(fs);
                }
                if (fs == 0 && stateMap.get(fs).nextStates.get(nk) == null)
                    Failure.put(ns, 0);
                else
                    Failure.put(ns, stateMap.get(fs).nextStates.get(nk));
                outputAdd(ns, Output.get(Failure.get(ns)));
            }
        }
    }

    protected boolean getMatchChar(state currState, char cc, boolean isCaseSensitive) {
        boolean matchChar;
        if (isCaseSensitive) {
            if (currState.nextStates.containsKey(cc) == false)
                matchChar = false;
            else
                matchChar = true;
        } else {
            if (currState.nextStates.containsKey(Character.toLowerCase(cc)) == false && currState.nextStates.containsKey(Character.toUpperCase(cc)) == false)
                matchChar = false;
            else
                matchChar = true;
        }
        return matchChar;
    }

    protected LinkedList<SearchResult> search(String searchText, boolean isCaseSensitive) {
        LinkedList<SearchResult> out = new LinkedList<>();

        state currState = zeroState;
        for (int i = 0; i < searchText.length(); i++) {
            char cc = searchText.charAt(i);
            while (currState.getId() != 0 && !getMatchChar(currState, cc, isCaseSensitive)) {
                if (currState.getId() == 0)
                    currState = zeroState;
                else
                    currState = stateMap.get(Failure.get(currState.getId()));
            }

            if (getMatchChar(currState, cc, isCaseSensitive)) {
                state tmpState = stateMap.get(currState.nextStates.get(cc));
                if (!isCaseSensitive && tmpState == null) {
                    if (Character.isLowerCase(cc))
                        currState = stateMap.get(currState.nextStates.get(Character.toUpperCase(cc)));
                    else
                        currState = stateMap.get(currState.nextStates.get(Character.toLowerCase(cc)));
                } else
                    currState = tmpState;
            } else {
                if (currState.getId() == 0)
                    currState = zeroState;
            }
            if (Output.containsKey(currState.getId())) {
                LinkedList<TaggedString> matches = Output.get(currState.getId());
                for (TaggedString m : matches) {
                    out.add(new SearchResult(m,i - m.getString().length() + 1));
                }
            }
        }
        return out;
    }
}