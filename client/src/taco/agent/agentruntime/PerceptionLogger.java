package taco.agent.agentruntime;

import com.google.gson.Gson;
import hso.autonomy.agent.communication.perception.IPerceptor;
import taco.agent.model.agentmeta.impl.CarMetaModelVersion;

import java.io.*;
import java.text.SimpleDateFormat;
import java.util.Date;
import java.util.Map;

public class PerceptionLogger
{
	private final CarMetaModelVersion version;

	private final String scenario;

	private File outputFile;

	private BufferedWriter writer;

	public PerceptionLogger(CarMetaModelVersion version, String scenario)
	{
		this.version = version;
		this.scenario = scenario;
	}

	public void start()
	{
		// use current date/time as filename
		String filename = new SimpleDateFormat("yyyy-MM-dd hh-mm-ss").format(new Date()) + ".log";
		try {
			outputFile = new File("log/", filename);
			// create temporary log folder to save the logs
			if (!outputFile.getParentFile().exists() && !outputFile.getParentFile().mkdir()) {
				throw new IOException("Failed to create directory: " + outputFile.getParent());
			}
			writer = new BufferedWriter(new FileWriter(outputFile));
			write(new LogFileHeader(version.name(), scenario));
		} catch (IOException e) {
			e.printStackTrace();
		}
	}

	private void write(Object message) throws IOException
	{
		writer.write(new Gson().toJson(message) + "\n");
	}

	public void log(Map<String, IPerceptor> message)
	{
		if (writer != null) {
			try {
				write(message);
			} catch (IOException e) {
				e.printStackTrace();
			}
		}
	}

	public void stop()
	{
		if (writer != null) {
			try {
				writer.flush();
				writer.close();
			} catch (IOException e) {
				e.printStackTrace();
			}
		}
	}

	@SuppressWarnings("unused")
	public class LogFileHeader
	{
		public final String version;

		public final String scenario;

		public LogFileHeader(String version, String scenario)
		{
			this.version = version;
			this.scenario = scenario;
		}
	}
}
