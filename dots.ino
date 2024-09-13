#include <TFT_eSPI.h>
#include <math.h>
#include "esp_task_wdt.h" // Include for Watchdog Timer handling

TFT_eSPI tft = TFT_eSPI(135, 240); // Adjusted for 135x240 resolution

struct Point
{
  float x, y, z;
  float cx, cy, cz;
  int col;
};

Point pt[27]; // Ensure array size matches the number of points
float t = 0;

// Colors (adjust to your setup)
uint16_t colors[] = {
    TFT_WHITE, TFT_RED, TFT_GREEN, TFT_BLUE, TFT_YELLOW, TFT_CYAN, TFT_MAGENTA, TFT_ORANGE};

void setup()
{
  Serial.begin(115200); // For debugging output
  tft.begin();
  tft.setRotation(0); // Portrait mode
  tft.fillScreen(TFT_BLACK);

  // Initialize points
  int idx = 0;
  for (float y = -1; y <= 1; y += 1.0 / 3)
  {
    for (float x = -1; x <= 1; x += 1.0 / 3)
    {
      for (float z = -1; z <= 1; z += 1.0 / 3)
      {
        if (idx < 27)
        { // Ensure we stay within array bounds
          pt[idx].x = x;
          pt[idx].y = y;
          pt[idx].z = z;
          pt[idx].col = 8 + (int(x * 2 + y * 3) % 8);
          idx++;
        }
      }
    }
  }

  // Proper Watchdog Timer Initialization
  esp_task_wdt_config_t wdt_config = {
      .timeout_ms = 10000,                             // 10 seconds
      .idle_core_mask = (1 << portNUM_PROCESSORS) - 1, // All cores
      .trigger_panic = true,
  };
  esp_task_wdt_init(&wdt_config); // Initialize the watchdog timer

  esp_task_wdt_add(NULL); // Add the current task to the watchdog
}

void checkMemory()
{
  Serial.print("Free heap: ");
  Serial.println(ESP.getFreeHeap()); // Print free heap memory to monitor memory usage
}

// Rotate point x,y by angle a
void rot(float x, float y, float a, float &rx, float &ry)
{
  rx = cos(a) * x - sin(a) * y;
  ry = cos(a) * y + sin(a) * x;
}

void loop()
{
  esp_task_wdt_reset(); // Feed the watchdog timer in each loop
  checkMemory();        // Monitor memory usage in each loop

  tft.fillScreen(TFT_BLACK); // Clear screen
  t += 0.05;                 // Increment time

  // Transform and rotate points
  for (int i = 0; i < 27; i++)
  {
    float cx, cz;
    rot(pt[i].x, pt[i].z, t / 8, cx, cz);
    pt[i].cx = cx;
    pt[i].cz = cz;

    rot(pt[i].y, pt[i].cz, t / 7, pt[i].cy, pt[i].cz);

    pt[i].cz += 2 + cos(t / 6);
  }

  // Sort points (bubble sort, furthest first)
  for (int pass = 0; pass < 4; pass++)
  {
    for (int i = 0; i < 26; i++)
    { // Array bounds check
      if (pt[i].cz < pt[i + 1].cz)
      {
        Point temp = pt[i];
        pt[i] = pt[i + 1];
        pt[i + 1] = temp;
      }
    }
  }

  // Draw points
  float rad1 = 5 + cos(t / 4) * 4;
  for (int i = 0; i < 27; i++)
  {
    float sx = 67 + pt[i].cx * 64 / pt[i].cz; // Adjusted for 135x240 screen
    float sy = 120 + pt[i].cy * 64 / pt[i].cz;
    float rad = rad1 / pt[i].cz;

    if (pt[i].cz > 0.1)
    {
      tft.fillCircle(sx, sy, rad, colors[pt[i].col % 8]);
      tft.fillCircle(sx + rad / 3, sy - rad / 3, rad / 3, TFT_WHITE);
    }
  }

  delay(16); // Roughly 60 FPS
}
