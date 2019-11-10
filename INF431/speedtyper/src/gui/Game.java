package gui;

import javax.swing.*;
import java.awt.*;
import java.awt.event.*;
import java.lang.String;
import java.text.SimpleDateFormat;
import java.util.LinkedList;
import java.util.List;
import java.util.concurrent.BlockingQueue;
import java.util.concurrent.LinkedBlockingQueue;
import java.util.concurrent.atomic.AtomicInteger;
import java.awt.event.KeyEvent;
import java.awt.event.KeyListener;
import backend.*;

public class Game extends BigPanel implements ActionListener, KeyListener {

	private final long duration = 30000;

	private final SimpleDateFormat timeFormat = new SimpleDateFormat("mm:ss");
	private final String stringStart = "Start";
	private final String stringStop = "Stop";
	private JLabel labelTitle;
	private JLabel labelName;
	private JLabel labelTime;
	private JLabel labelBestScore;
	private JLabel labelBestScoreValue;
	private JLabel labelCurrentScore;
	public JLabel labelCurrentScoreValue;
	private JLabel labelSynonym;
	private JLabel labelSynonymValue;
	private JTextArea typedWords;
	private JLabel labelIncorrectWords;
	private JTextArea incorrectWords;
	private JButton button;
	private Timer countdownTimer;
	private long currentDuration = duration;

	private AtomicInteger score = new AtomicInteger(0);
	private String tmpString = "";

	private LinkedList<String> sentences;
	private BlockingQueue<LinkedList<String>> filasentences;
	private BlockingQueue<Integer> filaScores;
	private BlockingQueue<String> filaSynonyms;
	private BlockingQueue<String> filaWrongWords;
	private WordHandler wordHandler;
	private ScoreEditor scoreEditor;
	private SynonymEditor synonymEditor;
	private WrongWordsEditor wrongWordsEditor;
	private Thread t;

	public Game() {
		game = this;
		sentences = new LinkedList<String>();
		filasentences = new LinkedBlockingQueue<LinkedList<String>>();
		filaScores = new LinkedBlockingQueue<Integer>();
		filaSynonyms = new LinkedBlockingQueue<String>();
		filaWrongWords = new LinkedBlockingQueue<String>();
		scoreEditor = new ScoreEditor();
		synonymEditor = new SynonymEditor();
		wrongWordsEditor = new WrongWordsEditor();
		wordHandler = new WordHandler(filasentences, score, filaScores, filaSynonyms, filaWrongWords);
		t = new Thread(wordHandler);
		t.start();
		scoreEditor.execute();
		synonymEditor.execute();
		wrongWordsEditor.execute();
	}

	public JPanel createComponents() {
		JPanel pane = new JPanel();
		pane.setLayout(new GridBagLayout());
		pane.setBackground(backgroundColor);
		pane.setBorder(BorderFactory.createCompoundBorder(BorderFactory.createLineBorder(fontColor, 4),
				BorderFactory.createEmptyBorder(20, 40, 20, 40)));
		GridBagConstraints gbc;

		// Title
		gbc = createGbc(0, 0);
		gbc.gridwidth = 4;
		gbc.fill = GridBagConstraints.BOTH;
		gbc.anchor = GridBagConstraints.CENTER;
		labelTitle = setFont("Speed-Typer", fontTitle);
		labelTitle.setHorizontalAlignment(SwingConstants.CENTER);
		pane.add(labelTitle, gbc);

		// Name
		gbc = createGbc(0, 1);
		labelName = setFont(username, fontName);
		pane.add(labelName, gbc);

		// Timer
		gbc = createGbc(3, 1);
		countdownTimer = new Timer(1000, this);
		countdownTimer.setInitialDelay(0);
		labelTime = setFont(timeFormat.format(duration), fontName);
		labelTime.setHorizontalAlignment(SwingConstants.RIGHT);
		pane.add(labelTime, gbc);

		// Best Score Label
		gbc = createGbc(0, 2);
		gbc.fill = GridBagConstraints.BOTH;
		gbc.anchor = GridBagConstraints.CENTER;
		labelBestScore = setFont("Best Score", font);
		labelBestScore.setHorizontalAlignment(SwingConstants.LEFT);
		pane.add(labelBestScore, gbc);

		// Best Score Value
		gbc = createGbc(3, 2);
		labelBestScoreValue = setFont(Integer.toString(bestScore), font);
		labelBestScoreValue.setHorizontalAlignment(SwingConstants.RIGHT);
		pane.add(labelBestScoreValue, gbc);

		// Score Label
		gbc = createGbc(0, 3);
		gbc.fill = GridBagConstraints.BOTH;
		gbc.anchor = GridBagConstraints.CENTER;
		labelCurrentScore = setFont("Score", font);
		labelCurrentScore.setHorizontalAlignment(SwingConstants.LEFT);
		pane.add(labelCurrentScore, gbc);

		// Score Value
		gbc = createGbc(3, 3);
		gbc.fill = GridBagConstraints.BOTH;
		gbc.anchor = GridBagConstraints.CENTER;
		labelCurrentScoreValue = setFont(Integer.toString(score.get()), font);
		labelCurrentScoreValue.setHorizontalAlignment(SwingConstants.RIGHT);
		pane.add(labelCurrentScoreValue, gbc);

		// Synonym
		gbc = createGbc(0, 4);
		gbc.fill = GridBagConstraints.BOTH;
		gbc.anchor = GridBagConstraints.CENTER;
		labelSynonym = setFont("Synonym", font);
		labelSynonym.setHorizontalAlignment(SwingConstants.LEFT);
		pane.add(labelSynonym, gbc);

		// Synonym Value
		gbc = createGbc(3, 4);
		labelSynonymValue = setFont("", font);
		labelSynonymValue.setHorizontalAlignment(SwingConstants.RIGHT);
		labelSynonymValue.setForeground(Color.RED);
		pane.add(labelSynonymValue, gbc);

		// Typed Words TextArea
		gbc = createGbc(0, 6);
		gbc.gridwidth = 4;
		typedWords = new JTextArea(7, 20);
		typedWords.setLineWrap(true);
		typedWords.addKeyListener(this);
		typedWords.setEditable(false);
		typedWords.setFont(fontTextArea);
		typedWords.setCaretColor(Color.white);
		typedWords.setForeground(Color.white);
		typedWords.setBackground(backgroundColor);
		typedWords.setMargin(new Insets(5, 5, 5, 5));
		typedWords.setTransferHandler(null);
		JScrollPane scrollText = new JScrollPane(typedWords);
		scrollText.setBorder(BorderFactory.createCompoundBorder(BorderFactory.createLineBorder(fontColor, 2),
				BorderFactory.createEmptyBorder(5, 5, 5, 5)));
		scrollText.setVerticalScrollBarPolicy(ScrollPaneConstants.VERTICAL_SCROLLBAR_ALWAYS);
		scrollText.setBackground(backgroundColor);
		typedWords.getInputMap().put(KeyStroke.getKeyStroke("BACK_SPACE"), "none");
		typedWords.getInputMap().put(KeyStroke.getKeyStroke("ENTER"), "none");
		pane.add(scrollText, gbc);

		// Incorrect Words Label
		gbc = createGbc(0, 7);
		labelIncorrectWords = setFont("Incorrect words", font);
		labelIncorrectWords.setHorizontalAlignment(SwingConstants.LEFT);
		pane.add(labelIncorrectWords, gbc);

		// Incorrect Words TextArea
		gbc = createGbc(0, 8);
		gbc.insets.set(5, 5, 10, 5);
		gbc.gridwidth = 4;
		incorrectWords = new JTextArea(3, 20);
		incorrectWords.setEditable(false);
		incorrectWords.setLineWrap(true);
		incorrectWords.setFont(fontTextArea);
		incorrectWords.setForeground(Color.white);
		incorrectWords.setBackground(backgroundColor);
		JScrollPane scrollInc = new JScrollPane(incorrectWords);
		scrollInc.setVerticalScrollBarPolicy(ScrollPaneConstants.VERTICAL_SCROLLBAR_ALWAYS);
		scrollInc.setBackground(backgroundColor);
		scrollInc.setBorder(BorderFactory.createCompoundBorder(BorderFactory.createLineBorder(fontColor, 2),
				BorderFactory.createEmptyBorder(5, 5, 5, 5)));
		pane.add(scrollInc, gbc);

		// Button start
		gbc = createGbc(0, 9);
		gbc.ipady = 7;
		gbc.gridwidth = 4;
		gbc.fill = GridBagConstraints.BOTH;
		gbc.anchor = GridBagConstraints.CENTER;
		button = new JButton(stringStart);
		button.setFont(font);
		button.setBorder(BorderFactory.createCompoundBorder(BorderFactory.createLineBorder(fontColor, 2),
				BorderFactory.createEmptyBorder(5, 5, 5, 5)));
		button.setForeground(fontColor);
		button.setBackground(backgroundColor);
		button.setHorizontalAlignment(SwingConstants.CENTER);
		button.addActionListener(this);
		pane.add(button, gbc);
		return pane;
	}

	public void actionPerformed(ActionEvent e) {
		if (e.getSource() == countdownTimer) {
			currentDuration -= 1000;
			labelTime.setText(timeFormat.format(currentDuration));
			if (currentDuration <= 0) {
				countdownTimer.stop();
				while (!wordHandler.isIdle){ try {
					Thread.sleep(100);
				} catch (InterruptedException e1) {
					// TODO Auto-generated catch block
					e1.printStackTrace();
				}};
				if (score.get() > bestScore) {
					bestScore = score.get();
					labelBestScoreValue.setText(Integer.toString(bestScore));
				}
				typedWords.setText("");
				typedWords.setEditable(false);
				incorrectWords.setText("");
				labelCurrentScoreValue.setText("0");
				score.set(0);
				currentDuration = duration;
				button.setText(stringStart);
				labelSynonymValue.setText("");
				wordHandler.synonym="";
				tmpString= "";
				
			}
		}

		if (e.getSource() == button) {
			if (button.getText() == stringStart) {
				if (!countdownTimer.isRunning()) {
					button.setText(stringStop);
					countdownTimer.start();
					typedWords.setEditable(true);
					typedWords.requestFocusInWindow();
				}
			} else {
				while (!wordHandler.isIdle){ try {
					Thread.sleep(100);
				} catch (InterruptedException e1) {
					// TODO Auto-generated catch block
					e1.printStackTrace();
				}};
				if (score.get() > bestScore) {
					bestScore = score.get();
					labelBestScoreValue.setText(Integer.toString(bestScore));
				}
				typedWords.setText("");
				incorrectWords.setText("");
				labelCurrentScoreValue.setText("0");
				score.set(0);
				button.setText(stringStart);
				countdownTimer.stop();
				typedWords.setEditable(false);
				currentDuration = duration;
				labelTime.setText(timeFormat.format(currentDuration));
				labelSynonymValue.setText("");
				wordHandler.synonym="";
				tmpString= "";
				sentences = new LinkedList<String>();
			}
		}
	}

	// Handle the key typed event from the text field
	public void keyTyped(KeyEvent e) {
		if (countdownTimer.isRunning()) {
			int id = e.getID();
			if (id == KeyEvent.KEY_TYPED) {
				char c = e.getKeyChar();
				if (c == ' ' && tmpString.length() != 0) {
					sentences.add(tmpString);
					tmpString = "";
				} else if (c == ' ' && tmpString.length() == 0) {
				} else if (c == '.' && !sentences.isEmpty() && tmpString.length() == 0) {
					try {
						filasentences.put(sentences);
					} catch (InterruptedException e1) {
						e1.printStackTrace();
					}
					sentences = new LinkedList<String>();
				} else if (c == '.' && tmpString.length() != 0) {
					sentences.add(tmpString);
					tmpString = "";
					try {
						filasentences.put(sentences);
					} catch (InterruptedException e1) {
						e1.printStackTrace();
					}
					sentences = new LinkedList<String>();
				} else if (Character.isLetter(c)) {
					c = Character.toLowerCase(c);
					tmpString = tmpString + c;
				}
			}
		}
	}

	public void keyPressed(KeyEvent e) {
		// DO nothing
	}

	public void keyReleased(KeyEvent e) {
		// Do nothing
	}

	class ScoreEditor extends SwingWorker<Integer, Integer> {
		int i = 0;

		@Override
		protected Integer doInBackground() throws Exception {
			while (true) {
				i = filaScores.take();
				publish(i);
			}
		}

		@Override
		protected void process(List<Integer> chunks) {
			for (int number : chunks) {
				labelCurrentScoreValue.setText(Integer.toString(number));
			}
		}
	}

	class SynonymEditor extends SwingWorker<Integer, String> {

		@Override
		protected Integer doInBackground() throws Exception {
			while (true) {
				String str = filaSynonyms.take();
				publish(str);
			}
		}

		@Override
		protected void process(List<String> chunks) {
			for (String str : chunks) {
				labelSynonymValue.setText(str);
			}
		}
	}

	class WrongWordsEditor extends SwingWorker<Integer, String> {

		@Override
		protected Integer doInBackground() throws Exception {
			while (true) {
				String str = filaWrongWords.take();
				publish(str);
			}
		}

		@Override
		protected void process(List<String> chunks) {
			for (String str : chunks) {
				incorrectWords.append(str);
			}
		}
	}
}
