package backend;

import java.util.ArrayList;
import java.util.Iterator;
import java.util.LinkedList;
import java.util.List;
import java.util.concurrent.BlockingQueue;
import java.util.concurrent.atomic.AtomicInteger;

import org.json.JSONArray;
import org.json.JSONException;
import org.json.JSONObject;

import com.mashape.unirest.http.HttpResponse;
import com.mashape.unirest.http.JsonNode;
import com.mashape.unirest.http.Unirest;
import com.mashape.unirest.http.exceptions.UnirestException;

public class WordHandler implements Runnable {
	private final BlockingQueue<LinkedList<String>> fila;
	private final BlockingQueue<Integer> filaScores;
	private final BlockingQueue<String> filaSynonyms;
	private final BlockingQueue<String> filawrongWords;
	private AtomicInteger score;
	private LinkedList<String> listStr;
	public String synonym = "";
	public boolean isIdle = true;

	public WordHandler(BlockingQueue<LinkedList<String>> filaSentences, AtomicInteger score,
			BlockingQueue<Integer> filaScores, BlockingQueue<String> filaSynonyms, BlockingQueue<String> filaWrongWords) {
		this.fila = filaSentences;
		this.score = score;
		this.filaScores = filaScores;
		this.filaSynonyms= filaSynonyms;
		this.filawrongWords=filaWrongWords;
	}

	@Override
	public void run() {
		int tmpScore = 0;
		String tmpStr = "";
		boolean synFlag = true;
		List<String> errors;
		while (true) {
			try {
				listStr = fila.take();
				isIdle = false;
			} catch (InterruptedException e) {
				e.printStackTrace();
			}
			for (String str : listStr) {
				System.out.println("Treating word: "+ str);
				tmpStr += "+" + str;
				tmpScore += Math.pow(str.length(), 2);
				if (!synonym.isEmpty()){
					if (synonym.equals(str))
						synFlag = true;
					}
				else
					synFlag=true;
					
			}
			try {
				errors = findError(tmpStr);
				tmpScore += errorScore(errors);
				for (String str : listStr) {
					if (!isError(str, errors)){
						synonym = generateSyn(str);
						break;
					}
				}
			} catch (UnirestException e) {
				// TODO Auto-generated catch block
				e.printStackTrace();
			} catch (JSONException e) {
				// TODO Auto-generated catch block
				e.printStackTrace();
			} catch (InterruptedException e) {
				// TODO Auto-generated catch block
				e.printStackTrace();
			}
			tmpStr = "";
			errors = new ArrayList<String>();
			if (!synFlag){
				System.out.println("Synonym " + synonym + " not present.");
				tmpScore -= (int) Math.pow(synonym.length(), 4);
			}
			synFlag = false;
			score.addAndGet(tmpScore);
			filaScores.add(score.get());
			tmpScore = 0;

			if (fila.isEmpty())
				isIdle = true;
		}
	}

	List<String> findError(String str) throws UnirestException, JSONException, InterruptedException {
		HttpResponse<JsonNode> request = Unirest.get("https://montanaflynn-spellcheck.p.mashape.com/check/?text=" + str)
				.header("X-Mashape-Key", "L5VcZQ16kYmshr7JJX8znH6IU3zop1coB5bjsnub24Vff3qsVp")
				.header("Accept", "application/json").asJson();
		// retrieve the parsed JSONObject from the response
		JSONObject myObj = request.getBody().getObject();
		JSONObject errors = myObj.getJSONObject("corrections");
		Iterator<?> keysToCopyIterator = errors.keys();
		List<String> keysList = new ArrayList<String>();
		while (keysToCopyIterator.hasNext()) {
			String key = (String) keysToCopyIterator.next();
			filawrongWords.put(key + " ");
			keysList.add(key);
		}
		return keysList;
	}

	int errorScore(List<String> keyList) {
		int result = 0;
		for (String str2 : keyList) {
			System.out.println("Wrong word found: " + str2);
			result -= (Math.pow(str2.length(), 2) + Math.pow(str2.length(), 3));
		}
		return result;
	}

	boolean isError(String str2, List<String> keyList) {
		for (String str3 : keyList)
			if (str3.compareTo(str2) == 1)
				return true;
		return false;
	}

	String generateSyn(String str2) throws UnirestException, JSONException, InterruptedException {
		String result;
		String key2="";
		System.out.println(str2);
		HttpResponse<JsonNode> response = Unirest.get("http://words.bighugelabs.com/api/2/2e90a62500db4131da3609ca2c3022b1/"+ str2+"/json")
				.asJson();
		JSONObject myobj =  response.getBody().getObject();
		Iterator<?> keysToCopyIterator = myobj.keys();
		while (keysToCopyIterator.hasNext()) {
			key2 = (String) keysToCopyIterator.next();
		}
		if (key2.isEmpty()){
			return "";
			}
		JSONArray arr = myobj.getJSONObject(key2).getJSONArray("syn");
		result=(String) arr.get(arr.length()-1);
		if(result.contains(" "))
			result = "";
		System.out.println("New synonym to " + str2 + " " + result);
		filaSynonyms.put(result);
		return result;
	}

}
