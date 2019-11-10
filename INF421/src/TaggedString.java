public class TaggedString {
    private final String str;
    private final int tag;

    public TaggedString(String str, int tag) {
        this.str = str;
        this.tag = tag;
    }
    public int hashCode() {
        return str.hashCode() + tag;
    }

    public boolean equals(Object obj) {
        if(obj.getClass() != this.getClass())
            return false;
        if(((TaggedString)obj).getString().compareTo(str) == 0 && ((TaggedString)obj).getTag() == tag)
            return true;
        return false;
    }

    public String getString(){
        return str;
    }
    public int getTag(){
        return tag;
    }
}