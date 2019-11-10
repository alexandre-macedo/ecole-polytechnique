package gui;
import java.io.IOException;
import java.io.InputStream;
import java.io.InputStreamReader;

import javax.swing.*;
import java.awt.*;


public class Rules extends BigPanel {
    public JButton buttonLogin;
    public JTextField txtUsername;
    private JLabel labelTitle;
    private final Font fontTexField = new Font(fontString, Font.PLAIN, 12);

    public Rules() {
        rules = this;
    }
    
    public JPanel createComponents() {
        JPanel pane = new JPanel();
        pane.setLayout(new GridBagLayout());
        pane.setBackground(backgroundColor);
        pane.setBorder(BorderFactory.createCompoundBorder(BorderFactory.createLineBorder(fontColor, 4),
                BorderFactory.createEmptyBorder(20, 40, 20, 40)));

        GridBagConstraints gbc = new GridBagConstraints();
        
     // Title
        gbc = createGbc(0, 0);
		gbc.gridwidth = 4;
		gbc.fill = GridBagConstraints.BOTH;
		gbc.anchor = GridBagConstraints.CENTER;
		labelTitle = setFont("Speed-Typer", fontTitle);
		labelTitle.setHorizontalAlignment(SwingConstants.CENTER);
		pane.add(labelTitle, gbc);
        
        // Rules table
        gbc = createGbc(0, 1);
        JTextArea textArea = new JTextArea();
        textArea.setColumns(20);
        textArea.setLineWrap(true);
        textArea.setRows(5);
        textArea.setWrapStyleWord(true);
        textArea.setEditable(false);
        textArea.setFont(font);
        textArea.setForeground(fontColor);
        textArea.setBackground(backgroundColor);
        InputStream in = getClass().getResourceAsStream("rules.txt");
        try {
            textArea.read(new InputStreamReader(in), null);
        } catch (IOException e) {
            e.printStackTrace();
        }
        pane.add(textArea, gbc);
        
        // Nickname Field
        gbc = createGbc(0, 2);
        gbc.gridwidth = 2;
        gbc.ipady = 10;
        txtUsername = new JTextField(20);
        txtUsername.setHorizontalAlignment(JTextField.CENTER);
        txtUsername.setFont(fontTexField);
        txtUsername.setCaretColor(Color.white);
        txtUsername.setDocument(new JTextFieldLimit(10));
        txtUsername.setForeground(Color.white);
        txtUsername.setBackground(backgroundColor);
        txtUsername.setBorder(BorderFactory.createCompoundBorder(BorderFactory.createLineBorder(fontColor, 2),
                    BorderFactory.createEmptyBorder(5, 5, 5, 5)));
        pane.add(txtUsername, gbc);

        // Start button
        gbc = createGbc(0, 4);
        gbc.ipady = 7;
        gbc.fill = GridBagConstraints.BOTH;
        gbc.anchor = GridBagConstraints.CENTER;
        buttonLogin = new JButton("Ok! Got it!");
        buttonLogin.setHorizontalAlignment(SwingConstants.CENTER);
        buttonLogin.setFont(font);
        buttonLogin.setBorder(BorderFactory.createCompoundBorder(BorderFactory.createLineBorder(fontColor, 2),
                    BorderFactory.createEmptyBorder(5, 5, 5, 5)));
        buttonLogin.setForeground(fontColor);
        buttonLogin.setBackground(backgroundColor);
        buttonLogin.addActionListener(this);
        pane.add(buttonLogin, gbc);

        return pane;
    }
}

