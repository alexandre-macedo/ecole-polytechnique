package gui;
import java.awt.Color;
import java.awt.Font;
import javax.swing.*;
import java.awt.*;
import java.awt.event.*;
import java.lang.String;	


public abstract class BigPanel implements ActionListener, Runnable, Component {
    public static String username;
    protected static int bestScore;
    public static JPanel wrapper;
    public static JFrame frame;
    public static Rules rules;
    public static Game game;

    protected static final String fontString = "TimesRoman";
    protected static final Color fontColor = new Color(255, 255, 204);
    protected static final Color backgroundColor = new Color(0, 0, 0);
    
    protected static final Font font = new Font(fontString, Font.BOLD, 16);
    protected static final Font fontName = new Font(fontString, Font.BOLD, 20);
    protected static final Font fontTitle = new Font(fontString, Font.BOLD, 22);
	protected static final Font fontTextArea = new Font(fontString, Font.PLAIN, 12);


    public BigPanel() {
    }

    protected void print(Object arg) {
        System.out.println(arg);
    }

    public void actionPerformed(ActionEvent e) {
        if(e.getSource() == rules.buttonLogin) {
            username = rules.txtUsername.getText();
                    // Create next Screen
                    wrapper.removeAll();
                    wrapper.add(game.createComponents());
                    wrapper.revalidate();
                    wrapper.repaint();
        }
    }

    protected JLabel setFont(String s, Font font) {
        JLabel label = new JLabel(s);
        label.setFont(font);
        label.setForeground(fontColor);
        label.setBackground(backgroundColor);
        label.setOpaque(true);
        return label;
    }

    protected GridBagConstraints createGbc(int x, int y) {
        GridBagConstraints gbc = new GridBagConstraints();
        gbc.gridx = x;
        gbc.gridy = y;
        gbc.gridwidth = 1;
        gbc.gridheight = 1;

        gbc.fill = (x == 0) ? GridBagConstraints.BOTH
            : GridBagConstraints.HORIZONTAL;

        gbc.insets = new Insets(5, 5, 5, 5);
        gbc.weightx = (x == 0) ? 0.1 : 1.0;
        gbc.weighty = 1.0;
        return gbc;
    }

    public void run () {
        // Wrapper
        wrapper = new JPanel(new GridBagLayout());
        wrapper.setBorder(BorderFactory.createEmptyBorder(20, 20, 20, 20));
        wrapper.setBackground(backgroundColor);
        
        // Main Frame
        frame = new JFrame("Speed-Typer");
        frame.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
        frame.setContentPane(wrapper);
        //frame.setResizable(false);
        
        JPanel content = rules.createComponents();
        frame.add(content);
        frame.pack();
        Dimension screenSize = Toolkit.getDefaultToolkit().getScreenSize();
        int height = screenSize.height * 2 / 3;
        int width = screenSize.width * 1 / 4;
        frame.setSize(new Dimension(width, height));
        frame.setLocationRelativeTo(null); //center of screen
        frame.setVisible(true);
    }
}
