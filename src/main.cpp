#include <WiFi.h>          // Gère la connexion Wi-Fi
#include <WebServer.h>     // Crée un serveur web

// Remplace par ton SSID et mot de passe
const char* ssid = "OMNES Education";
const char* password = "A{mA_dw4D";

WiFiServer server(80);

void setup() {
  Serial.begin(115200);
  WiFi.begin(ssid, password);

  // Attente de la connexion
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Connexion au WiFi...");
  }

  Serial.println("Connecté au WiFi !");
  Serial.println(WiFi.localIP());

  server.begin();
}

void loop() {
  WiFiClient client = server.available();
  if (client) {
    Serial.println("Client connecté");
    while (client.connected()) {
      if (client.available()) {
        String req = client.readStringUntil('\r');
        Serial.println(req);
        client.flush();

        // Réponse simple HTTP
        client.println("HTTP/1.1 200 OK");
        client.println("Content-type:text/html");
        client.println();
        client.println("<h1>ESP32 Web Server</h1>");
        client.println();
        break;
      }
    }
    client.stop();
    Serial.println("Client déconnecté");
  }
}
