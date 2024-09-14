#include <TFT_eSPI.h>
#include <math.h>
#include "esp_task_wdt.h" // Include for Watchdog Timer handling

TFT_eSPI tft = TFT_eSPI(135, 240);      // Adjusted for 135x240 resolution
TFT_eSprite sprite = TFT_eSprite(&tft); // Create a sprite for off-screen drawing

struct Point
{
  float x, y, z;
  float cx, cy, cz;
  int col;
};

// Pico-8 color palette in RGB565 (Black removed)
uint16_t pico8_colors[] = {
    0x18C3, // Dark Blue
    0x7C09, // Dark Purple
    0x016A, // Dark Green
    0xA9A4, // Brown
    0x52EA, // Dark Gray
    0xC618, // Light Gray
    0xFFFB, // White
    0xF810, // Red
    0xFD60, // Orange
    0xFFE0, // Yellow
    0x07E6, // Green
    0x2B7F, // Blue
    0x8418, // Lavender
    0xFE36, // Pink
    0xFF55  // Peach
};

Point pt[216]; // Adjusted array size for 6x6x6 grid (6*6*6 = 216 points)
float t = 0;

void setup()
{
  Serial.begin(115200); // For debugging output
  tft.begin();
  tft.setRotation(0); // Portrait mode
  tft.fillScreen(TFT_BLACK);

  // Initialize the sprite to match the screen size (135x240)
  sprite.createSprite(135, 240);

  // Initialize points for a 6x6x6 grid with better increments (spacing of 0.4)
  int idx = 0;
  for (float y = -1; y <= 1; y += 0.4) // 6 steps evenly spaced along y-axis
  {
    for (float x = -1; x <= 1; x += 0.4) // 6 steps evenly spaced along x-axis
    {
      for (float z = -1; z <= 1; z += 0.4) // 6 steps evenly spaced along z-axis
      {
        if (idx < 216)
        { // Ensure we stay within array bounds
          pt[idx].x = x;
          pt[idx].y = y;
          pt[idx].z = z;
          pt[idx].col = idx % 15; // Cycle through 15 Pico-8 colors
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

  sprite.fillSprite(TFT_BLACK); // Clear the sprite (instead of the screen)
  t += 0.05;                    // Increment time

  // Transform and rotate points
  for (int i = 0; i < 216; i++)
  {
    float cx, cz;
    rot(pt[i].x, pt[i].z, t / 8, cx, cz);
    pt[i].cx = cx;
    pt[i].cz = cz;

    rot(pt[i].y, pt[i].cz, t / 7, pt[i].cy, pt[i].cz);

    pt[i].cz += 2 + cos(t / 6);

    // Limit cz to avoid division by very small numbers
    if (pt[i].cz < 0.1)
    {
      pt[i].cz = 0.1;
    }
  }

  // Sort points (bubble sort, furthest first)
  for (int pass = 0; pass < 4; pass++)
  {
    for (int i = 0; i < 215; i++)
    { // Array bounds check
      if (pt[i].cz < pt[i + 1].cz)
      {
        Point temp = pt[i];
        pt[i] = pt[i + 1];
        pt[i + 1] = temp;
      }
    }
  }

  // Draw points in the sprite
  float rad1 = 5 + cos(t / 4) * 4;
  for (int i = 0; i < 216; i++)
  {
    float sx = 67 + pt[i].cx * 64 / pt[i].cz; // Adjusted for 135x240 screen
    float sy = 120 + pt[i].cy * 64 / pt[i].cz;
    float rad = rad1 / pt[i].cz;

    // Ensure points are within screen bounds
    if (sx > 0 && sx < 135 && sy > 0 && sy < 240)
    {
      sprite.fillCircle(sx, sy, rad, pico8_colors[pt[i].col % 15]); // Use Pico-8 colors
      sprite.fillCircle(sx + rad / 3, sy - rad / 3, rad / 3, TFT_WHITE);
    }
  }

  sprite.pushSprite(0, 0); // Push the sprite to the screen
}
