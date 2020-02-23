
import java.io.BufferedReader;
import java.io.File;
import java.io.FileNotFoundException;
import java.io.FileReader;
import java.io.IOException;
import java.io.PrintWriter;
import java.io.UnsupportedEncodingException;
import java.nio.charset.StandardCharsets;
import java.time.LocalDate;
import java.time.ZonedDateTime;
import java.time.format.DateTimeFormatter;
import java.time.format.FormatStyle;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Scanner;

import de.siegmar.fastcsv.reader.CsvParser;
import de.siegmar.fastcsv.reader.CsvReader;
import de.siegmar.fastcsv.reader.CsvRow;

public class AngleExporter {
	static ArrayList<Double> range = new ArrayList<Double>();
	static ArrayList<Double> angle = new ArrayList<Double>();
	
	
	//static ArrayList<String> paths = new ArrayList<String>();
	
	
	static String inputLocation = null,outputLocation = null, fileName=null, note=null, saveLocation=null;
	
	static boolean save = false;
	
	public static void main(String[] args) {
		//posLeft.add(0.0);
		//velLeft.add(0.0);
		//timeLeft.add(0.0);
		//posRight.add(0.0);
		//velRight.add(0.0);
		//timeRight.add(0.0);
		 
		 String st;
		 
		
	
		try(BufferedReader br = new BufferedReader(new FileReader("Settings.txt"))) {
		    
		    int r = 0;
			  while ((st = br.readLine()) != null) {
				  if(r==1) inputLocation =st;
				  else if(r==3) outputLocation=st;
				  else if(r==5) fileName = st;
				  else if(r==7) note = st;
				  else if(r==9)saveLocation = st;
				  //else if(r>4) {
				//	  paths.add(st);
				 // }
			  	r++;
			  } 
		
		} catch (FileNotFoundException e1) {
			// TODO Auto-generated catch block
			e1.printStackTrace();
		} catch (IOException e1) {
			// TODO Auto-generated catch block
			e1.printStackTrace();
		}
	
	
		System.out.println("Input: "+inputLocation);
		System.out.println("Output: "+outputLocation);
		System.out.println("Save: "+saveLocation);
		
		
		System.out.println("File Name: "+fileName);
		
		getValues(fileName);
		
		export();
			
			System.out.println("Done!");
			getUserInput("Press enter to close");

}
	
	
	public static String getUserInput(String prompt) {
		Scanner reader = new Scanner(System.in);  // Reading from System.in
		System.out.println(prompt);
		String input = "";
		
		
		
		input = reader.nextLine(); // if there is another number  
		
		reader.close();
		
		return input;
	}
	
	public static void getValues(String name) {
		
	
		File anglesFile = new File(inputLocation+"\\"+name+".csv");
		
		System.out.println("Reading " +name);
		CsvReader csvReader = new CsvReader();
		
		
		int count=0;
		try (CsvParser csvParser = csvReader.parse(anglesFile, StandardCharsets.UTF_8)) {
		    CsvRow row;
		    while ((row = csvParser.nextRow()) != null) {
		    	if(row.getOriginalLineNumber()!=1) {
		      //  System.out.println("Read line: " + row);
	    		range.add(Double.parseDouble(row.getField(1)));
	    		angle.add(Double.parseDouble(row.getField(2)));
		        //System.out.println("First column of line: " + row.getField(0));
	    		count++;
		    	}
		    }
		} catch (IOException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
		
		
		
		System.out.println("Read "+count+" lines.");
		//System.out.println("Last position: "+posLeft.get(posRight.size()-1)+", "+posRight.get(posLeft.size()-1));
	}
	
	

	public static void export() {
		//Output format convert
		
		
		System.out.println("Converting to output format.");
		
		ArrayList<String> output = new ArrayList<String>(Arrays.asList(
				"package frc.robot.ShooterTuning;",
				"import frc.lib.util.InterpolatingDouble;" ,
				"import frc.lib.util.InterpolatingTreeMap;",
				"public class HoodAngles {",
				"	public static InterpolatingTreeMap<InterpolatingDouble, InterpolatingDouble> kHoodAutoAimMap = new InterpolatingTreeMap<>();",
				" 	static {",
				"		/*"
				));
		String time = ZonedDateTime.now().format(DateTimeFormatter.RFC_1123_DATE_TIME);
		output.add("		"+time);
		
		output.add("		"+note);
		
		output.add("		(inch, angle)");
		
		output.add("		*/");
		
		for(int i=0;i<angle.size();i++) {
			output.add("		kHoodAutoAimMap.put(new InterpolatingDouble("+range.get(i)+"), new InterpolatingDouble("+angle.get(i)+"));");
		}
		
		output.add("	}");
		
		output.add("}");
		
		
		System.out.println("Writing java files.");
		
		PrintWriter writer;
		
		try {
			writer = new PrintWriter(outputLocation+"\\HoodAngles.java", "UTF-8");
			
			for(String j:output) {
				writer.println(j);
			}
			writer.close();
			
		} catch (FileNotFoundException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		} catch (UnsupportedEncodingException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
		
		
		  DateTimeFormatter formatter = DateTimeFormatter.ofPattern("yyyy-MM-dd-HH-mm-ss");
		  String saveName = ZonedDateTime.now().format(formatter);
		 // LocalDate parsedDate = LocalDate.parse(saveName, formatter);
		  saveName+="-"+note;
		PrintWriter saveWriter;
		
		try {
			saveWriter = new PrintWriter(saveLocation+"\\"+saveName+".txt", "UTF-8");
			
			for(String j:output) {
				saveWriter.println(j);
			}
			saveWriter.close();
			
		} catch (FileNotFoundException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		} catch (UnsupportedEncodingException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
		
	}
}
