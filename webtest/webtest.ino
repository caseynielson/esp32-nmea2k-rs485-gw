/*
 * webtest — absolute minimum WiFi AP + web server
 * No CAN, no RS485, no Serial, no logging.
 * If 192.168.4.1 responds: web server works, problem is elsewhere.
 * If it doesn't: something in the board/IDE setup is wrong.
 */

#include <WiFi.h>
#include <WebServer.h>

WebServer server(80);

void setup() {
    WiFi.softAP("webtest", "12345678");
    server.on("/", []() {
        server.send(200, "text/plain", "OK");
    });
    server.begin();
}

void loop() {
    server.handleClient();
}
