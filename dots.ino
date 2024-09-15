#include <math.h>
#include <stdlib.h>
#include <TFT_eSPI.h>
#include "esp_task_wdt.h"

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

#define GRID_SIZE (8 * 8 * 8) // Corrected macro definition for grid size
#define POINTS_PER_AXIS 8     // Define points along each axis (X, Y, Z)

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

// Linearly interpolate between two values (used for transitions)
// Renamed to customLerp to avoid conflict with std::lerp
float customLerp(float start, float end, float progress)
{
  return start + progress * (end - start);
}

// Initialize points in both cube and sphere spaces
void initializePoints()
{
  int idx = 0;

  // Calculate step size for 8 points along each axis (-1 to 1)
  float stepSize = 2.0 / (POINTS_PER_AXIS - 1); // Dynamically calculate step size

  // Initialize points in a cubic arrangement
  for (int i = 0; i < POINTS_PER_AXIS; i++)
  {
    for (int j = 0; j < POINTS_PER_AXIS; j++)
    {
      for (int k = 0; k < POINTS_PER_AXIS; k++)
      {
        if (idx < GRID_SIZE)
        {
          // Cube coordinates (range from -1 to 1 with calculated step size)
          pt[idx].cubeX = -1.0 + i * stepSize;
          pt[idx].cubeY = -1.0 + j * stepSize;
          pt[idx].cubeZ = -1.0 + k * stepSize;

          // Sphere coordinates (using spherical coordinates)
          float u = (float)rand() / RAND_MAX;
          float v = (float)rand() / RAND_MAX;
          float theta = 2 * M_PI * u;
          float phi = acos(2 * v - 1);

          // Scale the sphere radius to spread points more
          pt[idx].sphereX = sphereRadius * sin(phi) * cos(theta);
          pt[idx].sphereY = sphereRadius * sin(phi) * sin(theta);
          pt[idx].sphereZ = sphereRadius * cos(phi);

          // Assign initial positions to cube positions
          pt[idx].x = pt[idx].cubeX;
          pt[idx].y = pt[idx].cubeY;
          pt[idx].z = pt[idx].cubeZ;

          // Color assignment
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

  // Initialize the sprite to match the screen size
  sprite.createSprite(TFT_WIDTH, TFT_HEIGHT);

  // Initialize points in both cube and sphere spaces
  initializePoints();

  // Watchdog Timer initialization
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

    // Move elements that are greater than key.cz to one position ahead
    while (j >= 0 && arr[j].cz < key.cz)
    {
      arr[j + 1] = arr[j];
      j = j - 1;
    }
    arr[j + 1] = key;
  }
}

void updateTransformation()
{
  // If currently transforming, update the progress
  if (transforming)
  {
    transformProgress += 0.02; // Adjust for smoother/slower transformation

    if (transformProgress >= 1.0)
    {
      transformProgress = 1.0;
      transforming = false; // Transformation complete
    }

    // Interpolate between cube and sphere positions for each point
    for (int i = 0; i < GRID_SIZE; i++)
    {
      pt[i].x = customLerp(pt[i].cubeX, pt[i].sphereX, transformProgress);
      pt[i].y = customLerp(pt[i].cubeY, pt[i].sphereY, transformProgress);
      pt[i].z = customLerp(pt[i].cubeZ, pt[i].sphereZ, transformProgress);
    }
  }
  else
  {
    // Randomly decide to start a transformation
    if (random(0, 1000) < 10) // Low probability to trigger transformation
    {
      transforming = true;
      transformProgress = 0.0;

      // Swap the cube and sphere positions
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

  // Check if we need to switch between slow-motion and fast-motion
  if (t_mod >= nextChange)
  {
    // Randomly choose a duration for the next slow or fast motion
    nextChange = t_mod + random(200, 1000) / 100.0; // Random duration (2 to 10 seconds)

    // Randomly decide if we should switch to slow motion or fast motion
    if (random(0, 2) == 0)
    {
      inSlowMotion = true;
      currentSpeed = 0.5; // Very slow motion
    }
    else
    {
      inSlowMotion = false;
      currentSpeed = 5.0; // Fast motion
    }
  }

  // Modulate time increment based on the current speed factor (randomized pauses and bursts)
  float baseSpeed = 0.1;         // Base speed of time progression
  t_mod += baseSpeed;            // Always increment modulation time
  t += baseSpeed * currentSpeed; // Dynamic time increment based on current speed (slow or fast)

  // Update transformation between cube and sphere
  updateTransformation();

  // Precompute sine and cosine values for rotation (leave these unchanged)
  const float cos_t8 = cos(t / 4);
  const float sin_t8 = sin(t / 4);
  const float cos_t7 = cos(t / 6);
  const float sin_t7 = sin(t / 6);
  const float cos_t6 = cos(t / 5);

  // Transform and rotate points
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

  // Use insertion sort for depth sorting
  insertionSort(pt, GRID_SIZE);

  // Draw points in the sprite
  const float rad1 = 5 + cos(t / 4) * 4;
  for (int i = 0; i < GRID_SIZE; i++)
  {
    const float inv_cz = 1.0f / pt[i].cz;
    const float sx = TFT_WIDTH / 2 + pt[i].cx * 64 * inv_cz;
    const float sy = TFT_HEIGHT / 2 + pt[i].cy * 64 * inv_cz;
    const float rad = rad1 * inv_cz;

    if (sx > -rad && sx < TFT_WIDTH + rad && sy > -rad && sy < TFT_HEIGHT + rad && pt[i].cz > 0.1f)
    {
      sprite.fillCircle(sx, sy, rad, pico8_colors[pt[i].col % 15]); // Use Pico-8 colors
      sprite.fillCircle(sx + rad / 3, sy - rad / 3, rad / 3, TFT_WHITE);
    }
  }

  // Push the sprite to the screen
  sprite.pushSprite(0, 0);
}
