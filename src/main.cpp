#include <Arduino.h>
#include "config.h"
#include "drive_system.h"
#include "web_server.h"
#include "grid_control.h"

//Task Handle for Web Server
TaskHandle_t TaskWeb;

//Debug Timer Variables
unsigned long lastDebugTime = 0;
const int DEBUG_INTERVAL = 100; //print every 10ms

//Thread for Core 0: Web Server
void TaskWebCode(void * pvParameters)
{
  Serial.print("Web Server running on Core: ");
  Serial.println(xPortGetCoreID());

  for(;;)
  {
    //Handle WiFi/Web Client
    handleClient();
    //yield to prevent watchdog timer crash
    vTaskDelay(2 / portTICK_PERIOD_MS);
  }
}

//Main Setup Runs on Core 1
void setup()
{
  Serial.begin(115200);

  //Initialize Hardware
  initDriveSystem();
  initWebServer();

  //Create web server task on Core 0
  xTaskCreatePinnedToCore(
    TaskWebCode, //Function to Call
    "TaskWeb",   //Name of Task
    10000,       //Stack Size (bytes)
    NULL,        //Parameter
    1,           //Priority
    &TaskWeb,    //Task Handle
    0            //Core ID
    );

    Serial.print("Robot Logic running on Core: ");
    Serial.println(xPortGetCoreID());
}

void loop()
{
  handleGrid();
  updateDriveSystem();

  //debug print
  if (millis() - lastDebugTime >= DEBUG_INTERVAL)
  {
    lastDebugTime = millis();
    
    // Format: Label1:Value1,Label2:Value2,...
    Serial.print("RPM_L:");
    Serial.print(getCurrentRPMLeft());
    Serial.print(",");
    
    Serial.print("Target_L:");
    Serial.print(getTargetRPMLeft());
    Serial.print(",");
    
    Serial.print("RPM_R:");
    Serial.print(getCurrentRPMRight());
    Serial.print(",");
    
    Serial.print("Target_R:");
    Serial.println(getTargetRPMRight()); // println only at the very end
  }


  //Small Delay to prevent CPU saturation on core 1
  delay(1);
}