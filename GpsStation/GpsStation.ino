#include <Wire.h>
#include <SparkFun_u-blox_GNSS_Arduino_Library.h>
#define ARDUINOJSON_USE_DOUBLE 1
#include <ArduinoJson.h>
#include <WebServer.h>
#include <WiFi.h>
#include "secrets.h"
#include "index_html.h"
#include <Preferences.h>

#define JSON_CONTENT_TYPE "application/json"
#define FORM_CONTENT_TYPE "application/x-www-form-urlencoded"
#define HTML_CONTENT_TYPE "text/html"
#define GNSS_DATA_BUFFER_SIZE 16384
#define GNSS_DATA_CHUNK_SIZE 512
#define PREFERENCES_NAMESPACE "GnssStation"
#define RTCM_SEND_TIMEOUT_MS 10000

const char * GnssFixTypeName[] = {"no fix", "dead reckoning", "2D", "3D", "GNSS+reckoning", "time only"};
const char * GnssCarrierSolutionTypeName[] = { "no solution", "high precision, floating", "high precision fix" };

WebServer HttpServer(80);
SFE_UBLOX_GNSS GnssDevice;
Preferences preferences;
WiFiClient RtcmClient;

long RtcmDataCounter = 0;
long RawxDataCounter = 0;
long SfrbxDataCounter = 0;
long RtcmDataWritten = 0;

bool EnableRtcmReporting = false;
bool NewRtcmDataAvailable = false;
bool RtcmClientReady = false;
long TimeOfLastRtcmData;

long FileBytesSent = 0;
long SampleCompleteTimeOfWeek = 0;
long CurrentTimeOfWeek = 0;
byte ChunkBuffer[GNSS_DATA_CHUNK_SIZE+16];//add room for null


void setup()
{
  Serial.setDebugOutput(true);
  Serial.begin(115200);
  vTaskDelay(1000 / portTICK_PERIOD_MS);//let serial or other weirdness from a fast reboot settle in.
  if (!Wire.begin())
  {
	  Serial.println("Restarting in 5 seconds...");
	  vTaskDelay(5000 / portTICK_PERIOD_MS);//let slaves settle down. check serial for details.
	  ESP.restart();
  }
  if (!setupGnssDevice())
  {
	  Serial.println("Restarting in 5 seconds...");
	  vTaskDelay(5000 / portTICK_PERIOD_MS);//let slaves settle down. check serial for details.
	  ESP.restart();
  }
  preferences.begin(PREFERENCES_NAMESPACE);

  vTaskDelay(1000 / portTICK_PERIOD_MS);
  EnableRtcmReporting = preferences.getBool("CasterEnabled");

  xTaskCreate(	  
	  task_WiFi,	  
	  "WiFi",	  
	  4096,	  
	  NULL,	  
	  1,	  
	  NULL);
  xTaskCreate(
	  task_HttpServer,
	  "HttpServer",
	  4096,
	  NULL,
	  1,
	  NULL);
  xTaskCreate(
	  task_GnssDevice,
	  "GnssDevice",
	  4096,
	  NULL,
	  1,
	  NULL);

}

void loop()
{

}


#pragma region WIFI

void task_WiFi(void* parameter)
{
	while (true) {
		if (WiFi.status() != WL_CONNECTED)
		{
			connectToWiFi(WIFI_SSID, WIFI_PASSWORD);//from secrets.h
		}
		else {
			//Serial.print("WiFi Strength: ");
			//Serial.println(WiFi.RSSI());
		}
		vTaskDelay(5000 / portTICK_PERIOD_MS);
	}
}

void connectToWiFi(const char * ssid, const char * pwd)
{
  Serial.println("Connecting to WiFi network: " + String(ssid));

  
  WiFi.setAutoReconnect(true);
  WiFi.setHostname("GpsStation");
  WiFi.setSleep(false);
  WiFi.begin(ssid, pwd);

  int waitCounter = 0;

  while (WiFi.status() != WL_CONNECTED) 
  {
    vTaskDelay(500 / portTICK_PERIOD_MS);
    Serial.print(".");
	waitCounter++;
	if (waitCounter > 100)
	{
		ESP.restart();
	}
  }


  Serial.println();
  Serial.println("WiFi connected!");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());

  setupHttpServer();
}

#pragma endregion WIFI


#pragma region SERVER

void task_HttpServer(void* parameter)
{
	while (true)
	{
		if (WiFi.status() == WL_CONNECTED)
		{
			HttpServer.handleClient();
		}

		vTaskDelay(200 / portTICK_PERIOD_MS);
	}

}

void setupHttpServer()
{
	Serial.println("Setting up HTTP Server...");

	HttpServer.on("/system", HTTP_GET, handleGetSystem);
	HttpServer.on("/restart", HTTP_POST, handlePostRestart);
	HttpServer.on("/staticposition", HTTP_GET, handleGetStaticPosition);
	HttpServer.on("/staticposition", HTTP_POST, handlePostStaticPosition);
	HttpServer.on("/status", HTTP_GET, handleGetStatus);
	HttpServer.on("/rawdata.ubx", HTTP_GET, handleGetRawData);
	HttpServer.on("/caster", HTTP_GET, handleGetCaster);
	HttpServer.on("/caster", HTTP_POST, handlePostCaster);


	HttpServer.on("/", HTTP_GET, handleIndex);
	HttpServer.on("/index", HTTP_GET, handleIndex);
	HttpServer.on("/index.html", HTTP_GET, handleIndex);
	HttpServer.onNotFound(handleNotFound);

	HttpServer.enableCrossOrigin();
	HttpServer.begin();
	Serial.println("HTTP Server started!");
}

void handleIndex()
{
	Serial.println("GET index");

	HttpServer.send_P(200, HTML_CONTENT_TYPE, index_html);
}

void handleNotFound()
{
	Serial.print("endpoint not found: ");
	Serial.println(HttpServer.uri());
	HttpServer.send(404, HTML_CONTENT_TYPE, "<html>Not Found</html>");
}

void handleGetSystem()
{
	Serial.println("GET system:");

	const int jsonDocumentSize = 2*JSON_OBJECT_SIZE(4);
	StaticJsonDocument<jsonDocumentSize> responseDocument;
	responseDocument["RSSI"] = WiFi.RSSI();
	responseDocument["IP"] = WiFi.localIP();
	responseDocument["MAC"] = WiFi.macAddress();
	responseDocument["Uptime"] = millis() / 1000;
	sendResponseDocument(&responseDocument);
}

void handlePostRestart()
{
	Serial.println("POST /restart:");

	const int jsonDocumentSize = JSON_OBJECT_SIZE(1);
	StaticJsonDocument<jsonDocumentSize> responseDocument;
	responseDocument["RestartQueued"] = true;
	sendResponseDocument(&responseDocument);
	GnssDevice.hardReset();
	vTaskDelay(1000 / portTICK_PERIOD_MS);
	ESP.restart();
}

void handleGetStaticPosition()
{
	Serial.println("GET /staticposition:");

	const int jsonDocumentSize = 2*JSON_OBJECT_SIZE(4) + JSON_OBJECT_SIZE(1);//needs extra for key strings
	StaticJsonDocument<jsonDocumentSize> responseDocument;
	responseDocument["X"] = String(preferences.getDouble("StaticX"),4);
	responseDocument["Y"] = String(preferences.getDouble("StaticY"),4);
	responseDocument["Z"] = String(preferences.getDouble("StaticZ"),4);
	responseDocument["Acc"] = String(preferences.getDouble("StaticAcc"),4);
	sendResponseDocument(&responseDocument);
}

void handlePostStaticPosition()
{
	Serial.println("POST /staticposition:");

	if (HttpServer.hasArg("plain"))
	{
		const int jsonDocumentSize = 2*JSON_OBJECT_SIZE(4) + JSON_OBJECT_SIZE(1);//needs extra for key strings
		Serial.print("Json Document Memory Allocation: ");
		Serial.println(jsonDocumentSize);
		StaticJsonDocument<jsonDocumentSize> responseDocument;
		DeserializationError error = deserializeJson(responseDocument, HttpServer.arg("plain"));
		if (error.code() == error.Ok)
		{
			if (responseDocument.containsKey("X") && responseDocument.containsKey("Y") && responseDocument.containsKey("Z") && responseDocument.containsKey("Acc"))
			{
				if (responseDocument["X"].is<double>() && responseDocument["Y"].is<double>() && responseDocument["Z"].is<double>() && responseDocument["Acc"].is<double>())
				{
					preferences.putDouble("StaticX", responseDocument["X"].as<double>());
					preferences.putDouble("StaticY", responseDocument["Y"].as<double>());
					preferences.putDouble("StaticZ", responseDocument["Z"].as<double>());
					preferences.putDouble("StaticAcc", responseDocument["Acc"].as<double>());
					
					preferences.end();
					preferences.begin(PREFERENCES_NAMESPACE);

					//verify
					responseDocument.clear();
					responseDocument["X"] = String(preferences.getDouble("StaticX"), 4);
					responseDocument["Y"] = String(preferences.getDouble("StaticY"), 4);
					responseDocument["Z"] = String(preferences.getDouble("StaticZ"), 4);
					responseDocument["Acc"] = String(preferences.getDouble("StaticAcc"), 4);
					setupStationMode(preferences.getDouble("StaticX"), preferences.getDouble("StaticY"), preferences.getDouble("StaticZ"), preferences.getDouble("StaticAcc"));

					sendResponseDocument(&responseDocument);
				}
				else
				{
					HttpServer.send(400, HTML_CONTENT_TYPE, "<html>X, Y, Z parameters must be floating point numbers</html>");
					return;
				}
					
			}
			else {
				HttpServer.send(400, HTML_CONTENT_TYPE, "<html>Provided content must include exactly four parameters: X, Y, Z, Acc</html>");
				return;
			}
		}
		else {
			Serial.print("JSON Parse Error: ");
			Serial.println(error.c_str());
			HttpServer.send(400, HTML_CONTENT_TYPE, "<html>Unable to parse JSON</html>");
			return;
		}

	}
	else {
		HttpServer.send(400, HTML_CONTENT_TYPE, "<html>A body must be present</html>");
		return;
	}
}

void handleGetStatus()
{
	Serial.println("GET /status:");

	const int jsonDocumentSize = JSON_OBJECT_SIZE(3) + JSON_OBJECT_SIZE(6) + 2*JSON_OBJECT_SIZE(4) + JSON_OBJECT_SIZE(3);
	StaticJsonDocument<jsonDocumentSize> responseDocument;
	responseDocument["LLHA"]["Latitude"] = String(((double)GnssDevice.getLatitude())/10000000,8);
	responseDocument["LLHA"]["Longitude"] = String(((double)GnssDevice.getLongitude()) / 10000000, 8);
	responseDocument["LLHA"]["Height"] = ((double)GnssDevice.getAltitude())/1000;
	responseDocument["LLHA"]["Altitude"] = ((double)GnssDevice.getAltitudeMSL())/1000;
	responseDocument["LLHA"]["HorizontalAccuracy"] = ((double)GnssDevice.getHorizontalAccEst())/1000;
	responseDocument["LLHA"]["VerticalAccuracy"] = ((double)GnssDevice.getVerticalAccEst())/1000;
	Serial.println("Got LLHA");

	responseDocument["Reception"]["SatellitesInView"] = GnssDevice.getSIV();
	responseDocument["Reception"]["FixType"] = GnssFixTypeName[GnssDevice.getFixType()];
	responseDocument["Reception"]["Carrier"] = GnssCarrierSolutionTypeName[GnssDevice.getCarrierSolutionType()];
	Serial.println("Got Reception");

	if (GnssDevice.getNAVHPPOSECEF() && GnssDevice.packetUBXNAVHPPOSECEF != NULL)
	{
		responseDocument["ECEF"]["3dAccuracy"] = ((double)(GnssDevice.packetUBXNAVHPPOSECEF->data.pAcc))/10000;
		responseDocument["ECEF"]["X"] = String(((double)(GnssDevice.packetUBXNAVHPPOSECEF->data.ecefX))/100 + ((double)(GnssDevice.packetUBXNAVHPPOSECEF->data.ecefXHp)) / 10000, 4);
		responseDocument["ECEF"]["Y"] = String(((double)(GnssDevice.packetUBXNAVHPPOSECEF->data.ecefY))/100 + ((double)(GnssDevice.packetUBXNAVHPPOSECEF->data.ecefYHp)) / 10000, 4);
		responseDocument["ECEF"]["Z"] = String(((double)(GnssDevice.packetUBXNAVHPPOSECEF->data.ecefZ))/100 + ((double)(GnssDevice.packetUBXNAVHPPOSECEF->data.ecefZHp)) / 10000, 4);
		Serial.println("Got ECEF");
	}
	sendResponseDocument(&responseDocument);
}

void handleGetRawData()
{
	Serial.println("GET /rawdata.ubx:");

	if (HttpServer.hasArg("sampleduration"))
	{
		long sampleDuration = strtol(HttpServer.arg("sampleduration").c_str(), NULL, 10);
		if (sampleDuration > 0)
		{
			SampleCompleteTimeOfWeek = sampleDuration * 1000 + CurrentTimeOfWeek;
			if (SampleCompleteTimeOfWeek < CurrentTimeOfWeek)
			{
				HttpServer.send(400, HTML_CONTENT_TYPE, "<html>sample duration too long or current time is too close to the end of the GPS week</html>");
				return;
			}

			

			Serial.println("Beginning raw data file transfer...");
			String newline = "\r\n";
			String header = "HTTP/1.1 200 OK" + newline;
			header += "Content-Type: application/octet-stream" + newline;
			header += "Transfer-Encoding: chunked" + newline;
			//header += "Connection: close" + newline;
			header += newline;

			HttpServer.client().write(header.c_str()); 
			Serial.println("Header written...");

			setupMessaging(true);
			GnssDevice.clearFileBuffer();
			GnssDevice.logRXMRAWX(true);
			GnssDevice.logRXMSFRBX(true);
			int bytesAvailable;
			int bytesToWrite;
			FileBytesSent = 0;

			Serial.println("Begin write loop");
			while (CurrentTimeOfWeek < SampleCompleteTimeOfWeek && HttpServer.client().connected())
			{
				bytesAvailable = GnssDevice.fileBufferAvailable();
				while (bytesAvailable >= 16 && HttpServer.client().connected())
				{
					Serial.println("Bytes available: "+String(bytesAvailable));
					bytesToWrite = bytesAvailable;
					if (bytesToWrite > GNSS_DATA_CHUNK_SIZE)
					{
						bytesToWrite = GNSS_DATA_CHUNK_SIZE;
					}

					sprintf((char*)ChunkBuffer, "%X%s", bytesToWrite, newline);
					int chunkHeaderLength = strlen((char*)ChunkBuffer);

					//vTaskEnterCritical(&GnssLogBufferMutex);
					GnssDevice.extractFileBufferData(ChunkBuffer + chunkHeaderLength*sizeof(byte), bytesToWrite);
					//vTaskExitCritical(&GnssLogBufferMutex);
					
					ChunkBuffer[chunkHeaderLength + bytesToWrite] = newline[0];
					ChunkBuffer[chunkHeaderLength + bytesToWrite + 1] = newline[1];

					HttpServer.client().write(ChunkBuffer, chunkHeaderLength + bytesToWrite + 2);

					bytesAvailable -= bytesToWrite;
					FileBytesSent += bytesToWrite;
					Serial.println("Wrote " + String(bytesToWrite) + " / "+String(FileBytesSent)+" bytes to client file:");
				}
				vTaskDelay(100 / portTICK_PERIOD_MS);
			}
			HttpServer.client().write(("0" + newline + newline).c_str());//end chunk
			Serial.println("all data written");


			GnssDevice.logRXMRAWX(false);
			GnssDevice.logRXMSFRBX(false);
			setupMessaging(false);

		}
	}
	else {
		HttpServer.send(400, HTML_CONTENT_TYPE, "<html>does not include mandatory 'sampleduration' (seconds) query parameter</html>");
		return;
	}
}

void handleGetCaster()
{
	Serial.println("GET /caster:");
	const int jsonDocumentSize = 4*JSON_OBJECT_SIZE(6) + JSON_OBJECT_SIZE(1);//needs extra for key strings
	StaticJsonDocument<jsonDocumentSize> responseDocument;

	responseDocument["Host"] = preferences.getString("CasterHost");
	responseDocument["Port"] = preferences.getUShort("CasterPort");
	responseDocument["MountPoint"] = preferences.getString("CasterMount");
	responseDocument["Password"] = preferences.getString("CasterPassword");
	responseDocument["Server"] = preferences.getString("CasterServer");
	responseDocument["Enabled"] = preferences.getBool("CasterEnabled");

	sendResponseDocument(&responseDocument);
}

void handlePostCaster()
{
	Serial.println("POST /caster:");

	if (HttpServer.hasArg("plain"))
	{
		const int jsonDocumentSize = 4*JSON_OBJECT_SIZE(6) + JSON_OBJECT_SIZE(1);//needs extra for key strings
		Serial.print("Json Document Memory Allocation: ");
		Serial.println(jsonDocumentSize);
		StaticJsonDocument<jsonDocumentSize> responseDocument;
		DeserializationError error = deserializeJson(responseDocument, HttpServer.arg("plain"));
		if (error.code() == error.Ok)
		{
			if (responseDocument.containsKey("Host") && responseDocument.containsKey("Port") && responseDocument.containsKey("MountPoint") && responseDocument.containsKey("Password") && responseDocument.containsKey("Server") && responseDocument.containsKey("Enabled"))
			{
				if (responseDocument["Host"].is<String>() && responseDocument["Port"].is<uint16_t>() && responseDocument["MountPoint"].is<String>() && responseDocument["Password"].is<String>() && responseDocument["Server"].is<String>() && responseDocument["Enabled"].is<bool>())
				{
					preferences.putString("CasterHost", responseDocument["Host"].as<String>());
					preferences.putUShort("CasterPort", responseDocument["Port"].as<ushort>());
					preferences.putString("CasterMount", responseDocument["MountPoint"].as<String>());
					preferences.putString("CasterPassword", responseDocument["Password"].as<String>());
					preferences.putString("CasterServer", responseDocument["Server"].as<String>());
					preferences.putBool("CasterEnabled", responseDocument["Enabled"].as<bool>());

					preferences.end();
					preferences.begin(PREFERENCES_NAMESPACE);

					//verify
					responseDocument.clear();
					responseDocument["Host"] = preferences.getString("CasterHost");
					responseDocument["Port"] = preferences.getUShort("CasterPort");
					responseDocument["MountPoint"] = preferences.getString("CasterMount");
					responseDocument["Password"] = preferences.getString("CasterPassword");
					responseDocument["Server"] = preferences.getString("CasterServer");
					responseDocument["Enabled"] = preferences.getBool("CasterEnabled");

					EnableRtcmReporting = preferences.getBool("CasterEnabled");

					sendResponseDocument(&responseDocument);
				}
				else
				{
					HttpServer.send(400, HTML_CONTENT_TYPE, "<html>parameter type mismatch</html>");
					return;
				}

			}
			else {
				HttpServer.send(400, HTML_CONTENT_TYPE, "<html>Provided content must include exactly six parameters: Host, Port, MountPoint, Password, Server, Enabled</html>");
				return;
			}
		}
		else {
			Serial.print("JSON Parse Error: ");
			Serial.println(error.c_str());
			HttpServer.send(400, HTML_CONTENT_TYPE, "<html>Unable to parse JSON</html>");
			return;
		}

	}
	else {
		HttpServer.send(400, HTML_CONTENT_TYPE, "<html>A body must be present</html>");
		return;
	}
}

void sendResponseDocument(JsonDocument * responseDocument)
{
	String content;
	serializeJsonPretty(*responseDocument, content);
	HttpServer.send(200, JSON_CONTENT_TYPE, content);
	Serial.println(content);
	Serial.println();
}

#pragma endregion SERVER


#pragma region GNSS

void task_GnssDevice(void* parameter)
{
	while (true)
	{
		if (WiFi.status() == WL_CONNECTED)
		{
			GnssDevice.checkUblox();
			GnssDevice.checkCallbacks();
			//GnssDevice.getRXMRAWX();
			CurrentTimeOfWeek = GnssDevice.getTimeOfWeek();

			//Serial.println("TOW=" + String(CurrentTimeOfWeek) + "; RTCM=" + RtcmDataCounter + "; RAWX=" + RawxDataCounter + "; SFRBX=" + SfrbxDataCounter);


			//start new session cases
			if (!RtcmClient.connected() && EnableRtcmReporting && (TimeOfLastRtcmData + RTCM_SEND_TIMEOUT_MS) > millis())
			{
				setupRtcmClient(
					preferences.getString("CasterHost").c_str(), 
					preferences.getUShort("CasterPort"), 
					preferences.getString("CasterMount").c_str(),
					preferences.getString("CasterPassword").c_str(),
					preferences.getString("CasterServer").c_str()
				);
			}

			//stop session cases
			if (RtcmClient.connected() && (!EnableRtcmReporting || (TimeOfLastRtcmData + RTCM_SEND_TIMEOUT_MS) < millis()))
			{
				stopRtcmClient();
			}
		}

		vTaskDelay(500 / portTICK_PERIOD_MS);

	}
}

bool setupGnssDevice()
{
	Serial.println("Connecting GNSS Device...");
	bool success = true;
	GnssDevice.setFileBufferSize(GNSS_DATA_BUFFER_SIZE);
	Serial.println("File buffer allocated!");

	success &= GnssDevice.begin();
	success &= GnssDevice.setAutoRXMSFRBXcallback(&handleSfrbxData);
	success &= GnssDevice.setAutoRXMRAWXcallback(&handleRawxData);
	GnssDevice.disableUBX7Fcheck();//apparently necessary for rawx stuff

	if(success)
	{
		Serial.println("GnssDevice Started!");
		GnssDevice.factoryDefault(5000);//clear any weird settings


		setupMessaging(false);
		setupStationMode(preferences.getDouble("StaticX"), preferences.getDouble("StaticY"), preferences.getDouble("StaticZ"), preferences.getDouble("StaticAcc"));
		return true;
	}
	else {
		Serial.println("[E] Unable to start GNSS Device!");
		return false;
	}
}

void setupMessaging(bool enableRaw)
{
	bool success = true;
	success &= GnssDevice.setI2COutput(COM_TYPE_UBX | COM_TYPE_NMEA | COM_TYPE_RTCM3);
	success &= GnssDevice.setNavigationFrequency(1);

	success &= GnssDevice.disableNMEAMessage(UBX_NMEA_GGA, COM_PORT_I2C);
	success &= GnssDevice.disableNMEAMessage(UBX_NMEA_GSA, COM_PORT_I2C);
	success &= GnssDevice.disableNMEAMessage(UBX_NMEA_GSV, COM_PORT_I2C);
	success &= GnssDevice.disableNMEAMessage(UBX_NMEA_RMC, COM_PORT_I2C);
	success &= GnssDevice.disableNMEAMessage(UBX_NMEA_GST, COM_PORT_I2C);
	success &= GnssDevice.disableNMEAMessage(UBX_NMEA_GLL, COM_PORT_I2C);
	success &= GnssDevice.disableNMEAMessage(UBX_NMEA_VTG, COM_PORT_I2C);

	if (enableRaw)
	{
		success &= GnssDevice.enableMessage(UBX_CLASS_RXM, UBX_RXM_RAWX, COM_PORT_I2C);
		success &= GnssDevice.enableMessage(UBX_CLASS_RXM, UBX_RXM_RAWX, COM_PORT_USB);
		success &= GnssDevice.enableMessage(UBX_CLASS_RXM, UBX_RXM_SFRBX, COM_PORT_I2C);
		success &= GnssDevice.enableMessage(UBX_CLASS_RXM, UBX_RXM_SFRBX, COM_PORT_USB);
	}
	else {
		success &= GnssDevice.disableMessage(UBX_CLASS_RXM, UBX_RXM_RAWX, COM_PORT_I2C);
		success &= GnssDevice.disableMessage(UBX_CLASS_RXM, UBX_RXM_RAWX, COM_PORT_USB);
		success &= GnssDevice.disableMessage(UBX_CLASS_RXM, UBX_RXM_SFRBX, COM_PORT_I2C);
		success &= GnssDevice.disableMessage(UBX_CLASS_RXM, UBX_RXM_SFRBX, COM_PORT_USB);
	}

	success &= GnssDevice.enableRTCMmessage(UBX_RTCM_1005, COM_PORT_I2C, 1);
	success &= GnssDevice.enableRTCMmessage(UBX_RTCM_1074, COM_PORT_I2C, 1);
	success &= GnssDevice.enableRTCMmessage(UBX_RTCM_1084, COM_PORT_I2C, 1);
	success &= GnssDevice.enableRTCMmessage(UBX_RTCM_1094, COM_PORT_I2C, 1);
	success &= GnssDevice.enableRTCMmessage(UBX_RTCM_1124, COM_PORT_I2C, 1);
	success &= GnssDevice.enableRTCMmessage(UBX_RTCM_1230, COM_PORT_I2C, 10);

	if (success)
	{
		Serial.println("messages set!");
	}
	else {
		Serial.println("[E] Some messages failed to change.");
	}

	GnssDevice.setAutoNAVHPPOSECEF(true);
	GnssDevice.setAutoPVT(true);
}

void setupStationMode(double x, double y, double z, double acc)
{
	bool success;

	if (isnan(acc))
	{
		Serial.println("[E] acc is nan!");
	}

	if (isnan(x) || isnan(y) || isnan(z) || isnan(acc) || acc <= 0)
	{
		Serial.println("Disabling station mode"); 
		success = GnssDevice.setSurveyMode(0, 0, 0.0);
		
	}
	else {
		success = Serial.println("Setting static position to: " + String(x,8) + ", " + String(y, 8) + ", " + String(z, 8) + ": " + String(acc, 8));
		UBX_NAV_HPPOSECEF_data_t data;
		data.ecefX = (int32_t)(x * 100);
		data.ecefY = (int32_t)(y * 100);
		data.ecefZ = (int32_t)(z * 100);
		data.ecefXHp = (int8_t)((x * 100) - ((double)data.ecefX));
		data.ecefXHp = (int8_t)((y * 100) - ((double)data.ecefY));
		data.ecefXHp = (int8_t)((z * 100) - ((double)data.ecefZ));

		GnssDevice.setStaticPosition(data.ecefX,data.ecefXHp,data.ecefY, data.ecefYHp, data.ecefZ, data.ecefZHp);//does not support accuracy field...
	}

	if (success)
	{
		Serial.println("Mode changed.");
	}
	else {
		Serial.println("[E] mode failed to change.");
	}


}

void setupRtcmClient(const char* hostname, uint16_t port, const char* mountpoint, const char* mountpointpassword, const char* servername)
{
	Serial.printf("Connecting to %s on port %s", hostname, String(port));
	if (!RtcmClient.connect(hostname, port))
	{
		Serial.println("[E] Failed to initialize connection to ntrip server");
		return;
	}

	const int SERVER_BUFFER_SIZE = 512;
	char serverBuffer[SERVER_BUFFER_SIZE];
	snprintf(serverBuffer, SERVER_BUFFER_SIZE, "SOURCE %s /%s\r\nSource-Agent: NTRIP %s/%s\r\n\r\n",
		mountpointpassword, mountpoint, servername, "App Version 1.0");
	Serial.printf("Sending credentials:\n%s\n", serverBuffer);
	RtcmClient.write(serverBuffer, strlen(serverBuffer));

	//Wait for response
	unsigned long timeout = millis();
	while (RtcmClient.available() == 0)
	{
		if (millis() - timeout > 5000)
		{
			Serial.println("[E] Client Timeout while waiting for credential verification");
			RtcmClient.stop();
			return;
		}
		vTaskDelay(10 / portTICK_PERIOD_MS);
	}

	//Check reply
	bool connectionSuccess = false;
	char response[512];
	int responseSpot = 0;
	while (RtcmClient.available())
	{
		response[responseSpot++] = RtcmClient.read();
		if (strstr(response, "200") > 0) //Look for 'ICY 200 OK'
			connectionSuccess = true;
		if (responseSpot == 512 - 1) break;
	}
	response[responseSpot] = '\0';

	if (connectionSuccess == false)
	{
		Serial.printf("[E] Failed to receive good connection response from RTK2Go: %s", response);
		return;
	}

	RtcmDataWritten = 0;
	TimeOfLastRtcmData = millis();
	RtcmClientReady = true;
}

void stopRtcmClient()
{
	Serial.println("Stopping RTCM client");
	RtcmClientReady = false;
	RtcmDataWritten = 0;
	RtcmClient.stop();
}

void SFE_UBLOX_GNSS::processRTCM(uint8_t incoming)
{
	RtcmDataCounter++;
	NewRtcmDataAvailable = true;
	TimeOfLastRtcmData = millis();

	if (RtcmClientReady && RtcmClient.connected())
	{
		RtcmClient.write(incoming);
		RtcmDataWritten++;
	}
}

void handleRawxData(UBX_RXM_RAWX_data_t ubxDataStruct)
{
	RawxDataCounter++;
}

void handleSfrbxData(UBX_RXM_SFRBX_data_t ubxDataStruct)
{
	SfrbxDataCounter++;
}


#pragma endregion GNSS