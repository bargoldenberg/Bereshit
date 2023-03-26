package simulation;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.util.ArrayList;

public class CSVWriter {
    public void writeToCSV(ArrayList<ArrayList<String>> lines) throws IOException {
        File csvFile = new File("landing.csv");
        FileWriter fileWriter = new FileWriter(csvFile);
        for (ArrayList<String> data : lines) {
            StringBuilder line = new StringBuilder();
            for (int i = 0; i < data.size(); i++) {
                line.append("\"");
                line.append(data.get(i).replaceAll("\"", "\"\""));
                line.append("\"");
                if (i != data.size() - 1) {
                    line.append(',');
                }
            }
            line.append("\n");
            fileWriter.write(line.toString());
        }
        fileWriter.close();
    }
}
