#include <TFT_eSPI.h>
#include <math.h>
#include "esp_task_wdt.h" // Include for Watchdog Timer handling

#define TFT_WIDTH 135
#define TFT_HEIGHT 240

#define GRID_SIZE 6 * 6 * 6

TFT_eSPI tft = TFT_eSPI(TFT_WIDTH, TFT_HEIGHT); // Adjusted for 135x240 resolution
TFT_eSprite sprite = TFT_eSprite(&tft);         // Create a sprite for off-screen drawing

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

Point pt[GRID_SIZE]; // Adjusted array size for 6x6x6 grid (6*6*6 = GRID_SIZE points)
float t = 0;

void setup()
{
  Serial.begin(115200); // For debugging output
  tft.begin();
  tft.setRotation(0); // Portrait mode
  tft.fillScreen(TFT_BLACK);

  // Initialize the sprite to match the screen size (135x240)
  sprite.createSprite(TFT_WIDTH, TFT_HEIGHT);

  // Initialize points for a 6x6x6 grid
  int idx = 0;
  for (float y = -1; y <= 1; y += 0.4) // 6 steps evenly spaced along y-axis
  {
    for (float x = -1; x <= 1; x += 0.4) // 6 steps evenly spaced along x-axis
    {
      for (float z = -1; z <= 1; z += 0.4) // 6 steps evenly spaced along z-axis
      {
        if (idx < GRID_SIZE)
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

// Rotate point x,y by precomputed angle (optimization)
void rot(float x, float y, float cos_a, float sin_a, float &rx, float &ry)
{
  rx = cos_a * x - sin_a * y;
  ry = sin_a * x + cos_a * y;
}

// Insertion sort for faster sorting of points based on cz
void insertionSort(Point arr[], int n)
{
  for (int i = 1; i < n; i++)
  {
    Point key = arr[i];
    int j = i - 1;

    // Move elements of arr[0..i-1], that are greater than key.cz, to one position ahead of their current position
    while (j >= 0 && arr[j].cz < key.cz)
    {
      arr[j + 1] = arr[j];
      j = j - 1;
    }
    arr[j + 1] = key;
  }
}

void loop()
{
  esp_task_wdt_reset(); // Feed the watchdog timer in each loop
  checkMemory();        // Monitor memory usage in each loop

  sprite.fillSprite(TFT_BLACK); // Clear the sprite (instead of the screen)
  t += 0.05;                    // Increment time

  // Precompute sine and cosine values for rotation
  float cos_t8 = cos(t / 8);
  float sin_t8 = sin(t / 8);
  float cos_t7 = cos(t / 7);
  float sin_t7 = sin(t / 7);
  float cos_t6 = cos(t / 6);

  // Transform and rotate points
  for (int i = 0; i < GRID_SIZE; i++)
  {
    float cx, cz;
    rot(pt[i].x, pt[i].z, cos_t8, sin_t8, cx, cz);
    pt[i].cx = cx;
    pt[i].cz = cz;

    rot(pt[i].y, pt[i].cz, cos_t7, sin_t7, pt[i].cy, pt[i].cz);

    pt[i].cz += 2 + cos_t6;

    // Limit cz to avoid division by very small numbers
    if (pt[i].cz < 0.1)
    {
      pt[i].cz = 0.1;
    }
  }

  // Use insertion sort for faster sorting
  insertionSort(pt, GRID_SIZE);

  // Draw points in the sprite
  float rad1 = 5 + cos(t / 4) * 4;
  for (int i = 0; i < GRID_SIZE; i++)
  {
    float sx = TFT_WIDTH / 2 + pt[i].cx * 64 / pt[i].cz; // Adjusted for 135x240 screen
    float sy = TFT_HEIGHT / 2 + pt[i].cy * 64 / pt[i].cz;
    float rad = rad1 / pt[i].cz;

    // Ensure points are within screen bounds
    if (sx > 0 && sx < TFT_WIDTH && sy > 0 && sy < TFT_HEIGHT)
    {
      sprite.fillCircle(sx, sy, rad, pico8_colors[pt[i].col % 15]); // Use Pico-8 colors
      sprite.fillCircle(sx + rad / 3, sy - rad / 3, rad / 3, TFT_WHITE);
    }
  }

  sprite.pushSprite(0, 0); // Push the sprite to the screen
}
