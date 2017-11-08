package gui;


public class Starter {
    private BigPanel rules;
    @SuppressWarnings("unused")
	private BigPanel game;

    public Starter() {
        //init pointer for each screen
        rules = new Rules();
        game = new Game();
    }

    public void init() {
        Thread t = new Thread(rules);
        t.start();
    }
}
