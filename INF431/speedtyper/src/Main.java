import gui.Starter;
import com.mashape.unirest.http.exceptions.UnirestException;
public class Main {
	public static void main(String[] args) throws UnirestException {
		
		Starter starter = new Starter();
		starter.init();
	}
}
