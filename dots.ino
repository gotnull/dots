#include <math.h>
#include <stdlib.h>
#include <TFT_eSPI.h>
#include "esp_task_wdt.h"
#include <FastLED.h> // Include the FastLED library for Perlin noise

float t_mod = 0;                // Time variable for modulating speed
float pauseTime = 0;            // Time for current pause state
float nextChange = 0;           // When to change speed (randomized)
float currentSpeed = 1.0;       // Current speed factor
bool inSlowMotion = false;      // State variable for slow-motion
bool transforming = false;      // State for when transitioning between cube and sphere
float transformProgress = 0.0;  // Transition progress (0 to 1)
const float sphereRadius = 1.5; // Increase sphere radius to spread points more

// Inlined and optimized rotation calculation macro (saves function call overhead)
#define ROTATE(X, Y, COS_A, SIN_A, RX, RY) \
  RX = COS_A * X - SIN_A * Y;              \
  RY = SIN_A * X + COS_A * Y;

#define GRID_SIZE (6 * 6 * 6) // Corrected macro definition for grid size
#define POINTS_PER_AXIS 6     // Define points along each axis (X, Y, Z)

// Starfield parameters
#define NUM_STARS 100 // Number of stars in the starfield

#define TFT_WIDTH 170
#define TFT_HEIGHT 320

TFT_eSPI tft = TFT_eSPI(TFT_WIDTH, TFT_HEIGHT);
TFT_eSprite sprite = TFT_eSprite(&tft);

struct Point
{
  float x, y, z;    // Current position
  float cx, cy, cz; // Transformed position
  int col;          // Color

  float cubeX, cubeY, cubeZ;       // Position in cube space
  float sphereX, sphereY, sphereZ; // Position in sphere space
};

// Pico-8 color palette in RGB565 (Black removed)
const uint16_t pico8_colors[] = {
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

Point pt[GRID_SIZE]; // Array size for 8x8x8 grid (8*8*8 = GRID_SIZE points)
float t = 0;

struct Star
{
  float x, y, z; // Position of the star
};

// Array to hold the stars
Star stars[NUM_STARS];

// Initialize the stars with random positions and depths
void initializeStars()
{
  for (int i = 0; i < NUM_STARS; i++)
  {
    stars[i].x = (float)rand() / RAND_MAX * 2 - 1; // X between -1 and 1
    stars[i].y = (float)rand() / RAND_MAX * 2 - 1; // Y between -1 and 1
    stars[i].z = (float)rand() / RAND_MAX;         // Z between 0 and 1
  }
}

// Update starfield positions and reset stars if they move off the screen
void updateStars()
{
  for (int i = 0; i < NUM_STARS; i++)
  {
    stars[i].z -= 0.02; // Move stars towards the screen (increase speed if desired)

    if (stars[i].z <= 0) // Reset star if it reaches the viewer
    {
      stars[i].x = (float)rand() / RAND_MAX * 2 - 1; // X between -1 and 1
      stars[i].y = (float)rand() / RAND_MAX * 2 - 1; // Y between -1 and 1
      stars[i].z = 1.0;                              // Reset Z to 1 (far away)
    }
  }
}

// Draw the starfield
void drawStars()
{
  for (int i = 0; i < NUM_STARS; i++)
  {
    // Project stars onto the screen
    int sx = TFT_WIDTH / 2 + stars[i].x * (TFT_WIDTH / 2) / stars[i].z;
    int sy = TFT_HEIGHT / 2 + stars[i].y * (TFT_HEIGHT / 2) / stars[i].z;

    // Calculate brightness based on distance (closer stars are brighter)
    uint16_t brightness = 255 * (1 - stars[i].z);

    // Draw the star as a small point
    if (sx >= 0 && sx < TFT_WIDTH && sy >= 0 && sy < TFT_HEIGHT)
    {
      sprite.drawPixel(sx, sy, tft.color565(brightness, brightness, brightness));
    }
  }
}

// Linearly interpolate between two values (used for transitions)
float customLerp(float start, float end, float progress)
{
  return start + progress * (end - start);
}

// Initialize points in both cube and sphere spaces
void initializePoints()
{
  int idx = 0;

  float stepSize = 2.0 / (POINTS_PER_AXIS - 1); // Dynamically calculate step size

  for (int i = 0; i < POINTS_PER_AXIS; i++)
  {
    for (int j = 0; j < POINTS_PER_AXIS; j++)
    {
      for (int k = 0; k < POINTS_PER_AXIS; k++)
      {
        if (idx < GRID_SIZE)
        {
          pt[idx].cubeX = -1.0 + i * stepSize;
          pt[idx].cubeY = -1.0 + j * stepSize;
          pt[idx].cubeZ = -1.0 + k * stepSize;

          float u = (float)rand() / RAND_MAX;
          float v = (float)rand() / RAND_MAX;
          float theta = 2 * M_PI * u;
          float phi = acos(2 * v - 1);

          pt[idx].sphereX = sphereRadius * sin(phi) * cos(theta);
          pt[idx].sphereY = sphereRadius * sin(phi) * sin(theta);
          pt[idx].sphereZ = sphereRadius * cos(phi);

          pt[idx].x = pt[idx].cubeX;
          pt[idx].y = pt[idx].cubeY;
          pt[idx].z = pt[idx].cubeZ;

          pt[idx].col = 8 + (int(pt[idx].cubeX * 2 + pt[idx].cubeY * 3) % 8);
          idx++;
        }
      }
    }
  }
}

void setup()
{
  Serial.begin(115200); // For debugging output
  tft.begin();
  tft.setRotation(0); // Portrait mode
  tft.fillScreen(TFT_BLACK);

  sprite.createSprite(TFT_WIDTH, TFT_HEIGHT);

  initializePoints();
  initializeStars(); // Initialize the starfield

  esp_task_wdt_config_t wdt_config = {
      .timeout_ms = 10000,                             // 10 seconds
      .idle_core_mask = (1 << portNUM_PROCESSORS) - 1, // All cores
      .trigger_panic = true,
  };
  esp_task_wdt_init(&wdt_config); // Initialize the watchdog timer
  esp_task_wdt_add(NULL);         // Add the current task to the watchdog
}

// Optimized insertion sort
void insertionSort(Point arr[], int n)
{
  for (int i = 1; i < n; i++)
  {
    Point key = arr[i];
    int j = i - 1;
    while (j >= 0 && arr[j].cz < key.cz)
    {
      arr[j + 1] = arr[j];
      j = j - 1;
    }
    arr[j + 1] = key;
  }
}

void applyNoiseToPoints(float noiseIntensity, float t)
{
  for (int i = 0; i < GRID_SIZE; i++)
  {
    float noiseX = inoise8(pt[i].x * 50 + t, pt[i].y * 50 + t, pt[i].z * 50 + t);
    float noiseY = inoise8(pt[i].x * 50 + 100 + t, pt[i].y * 50 + 100 + t, pt[i].z * 50 + 100 + t);
    float noiseZ = inoise8(pt[i].x * 50 + 200 + t, pt[i].y * 50 + 200 + t, pt[i].z * 50 + 200 + t);

    float nx = map(noiseX, 0, 255, -100, 100) / 100.0;
    float ny = map(noiseY, 0, 255, -100, 100) / 100.0;
    float nz = map(noiseZ, 0, 255, -100, 100) / 100.0;

    pt[i].x += nx * noiseIntensity;
    pt[i].y += ny * noiseIntensity;
    pt[i].z += nz * noiseIntensity;
  }
}

void updateTransformation()
{
  if (transforming)
  {
    transformProgress += 0.02;
    if (transformProgress >= 1.0)
    {
      transformProgress = 1.0;
      transforming = false;
    }
    for (int i = 0; i < GRID_SIZE; i++)
    {
      pt[i].x = customLerp(pt[i].cubeX, pt[i].sphereX, transformProgress);
      pt[i].y = customLerp(pt[i].cubeY, pt[i].sphereY, transformProgress);
      pt[i].z = customLerp(pt[i].cubeZ, pt[i].sphereZ, transformProgress);
    }
    float noiseIntensity = sin(transformProgress * M_PI);
    applyNoiseToPoints(noiseIntensity * 0.2, t);
  }
  else
  {
    if (random(0, 1000) < 10)
    {
      transforming = true;
      transformProgress = 0.0;
      for (int i = 0; i < GRID_SIZE; i++)
      {
        float tempX = pt[i].cubeX;
        float tempY = pt[i].cubeY;
        float tempZ = pt[i].cubeZ;

        pt[i].cubeX = pt[i].sphereX;
        pt[i].cubeY = pt[i].sphereY;
        pt[i].cubeZ = pt[i].sphereZ;

        pt[i].sphereX = tempX;
        pt[i].sphereY = tempY;
        pt[i].sphereZ = tempZ;
      }
    }
  }
}

void loop()
{
  esp_task_wdt_reset(); // Feed the watchdog timer in each loop

  sprite.fillSprite(TFT_BLACK); // Clear the sprite (instead of clearing the entire screen)

  updateStars(); // Update the starfield
  drawStars();   // Draw the starfield

  // Check if we need to switch between slow-motion and fast-motion
  if (t_mod >= nextChange)
  {
    nextChange = t_mod + random(200, 1000) / 100.0;
    inSlowMotion = random(0, 2) == 0;
    currentSpeed = inSlowMotion ? 0.5 : 5.0;
  }

  float baseSpeed = 0.1;
  t_mod += baseSpeed;
  t += baseSpeed * currentSpeed;

  updateTransformation();

  const float cos_t8 = cos(t / 4);
  const float sin_t8 = sin(t / 4);
  const float cos_t7 = cos(t / 6);
  const float sin_t7 = sin(t / 6);
  const float cos_t6 = cos(t / 5);

  for (int i = 0; i < GRID_SIZE; i++)
  {
    float cx, cz;
    ROTATE(pt[i].x, pt[i].z, cos_t8, sin_t8, cx, cz);
    pt[i].cx = cx;
    pt[i].cz = cz;

    ROTATE(pt[i].y, pt[i].cz, cos_t7, sin_t7, pt[i].cy, pt[i].cz);

    pt[i].cz += 2 + cos_t6;

    if (pt[i].cz < 0.1f)
    {
      pt[i].cz = 0.1f;
    }
  }

  insertionSort(pt, GRID_SIZE);

  const float rad1 = 5 + cos(t / 4) * 4;
  for (int i = 0; i < GRID_SIZE; i++)
  {
    const float inv_cz = 1.0f / pt[i].cz;
    const float sx = TFT_WIDTH / 2 + pt[i].cx * 64 * inv_cz;
    const float sy = TFT_HEIGHT / 2 + pt[i].cy * 64 * inv_cz;
    const float rad = rad1 * inv_cz;

    if (sx > -rad && sx < TFT_WIDTH + rad && sy > -rad && sy < TFT_HEIGHT + rad && pt[i].cz > 0.1f)
    {
      sprite.fillCircle(sx, sy, rad, pico8_colors[pt[i].col % 15]);
      sprite.fillCircle(sx + rad / 3, sy - rad / 3, rad / 3, TFT_WHITE);
    }
  }

  sprite.pushSprite(0, 0);
}
