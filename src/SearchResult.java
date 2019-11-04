public class SearchResult {
        private TaggedString taggedString;
        private Integer matchPosition;

        public SearchResult(TaggedString taggedString, Integer second) {
            super();
            this.taggedString = taggedString;
            this.matchPosition = second;
        }

        public String toString()
        {
            return "(" + taggedString + ", " + matchPosition + ")";
        }

        public TaggedString getTaggedString() {
            return taggedString;
        }

        public Integer getMatchPosition() {
            return matchPosition;
        }
}
