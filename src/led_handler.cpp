#include "led_handler.h"

// We use 30 physical LEDs, but index 0 is sacrificial/off. 
// LIDAR_NUM_LEDS (29) represents the visible strip.
#define TOTAL_PHYSICAL_LEDS (LIDAR_NUM_LEDS + 1)

CRGB lidar_leds[TOTAL_PHYSICAL_LEDS];
CRGB active_base_color = CRGB::White; // Default base color
bool is_pulsing = false;

void led_init_early() {
    // Add LEDs for the total physical count
    FastLED.addLeds<LED_TYPE, LIDAR_LED_PIN, COLOR_ORDER>(lidar_leds, TOTAL_PHYSICAL_LEDS);
    FastLED.setBrightness(180);
    
    // Start with all LEDs off
    fill_solid(lidar_leds, TOTAL_PHYSICAL_LEDS, CRGB::Black);
    FastLED.show();
}

void led_animation() {
    // Animation: Light up from first and last LED (logical) towards the middle
    active_base_color = CRGB::White;
    for (int i = 0; i < LIDAR_NUM_LEDS / 2; i++) {
        // Shift index by +1 to skip the sacrificial pixel at physical index 0
        lidar_leds[i + 1] = active_base_color;
        lidar_leds[LIDAR_NUM_LEDS - i] = active_base_color; // (LIDAR_NUM_LEDS - 1 - i) + 1
        FastLED.show();
        delay(50);
    }
    
    // Ensure the center is filled for odd numbers
    if (LIDAR_NUM_LEDS % 2 != 0) {
        lidar_leds[(LIDAR_NUM_LEDS / 2) + 1] = active_base_color;
        FastLED.show();
    }
}

void led_set_ready() {
    is_pulsing = true;
}

void led_set_start() {
    is_pulsing = false;
    FastLED.setBrightness(120); // Reset to default brightness
}

void led_set_cyan() {
    active_base_color = CRGB::Cyan;
    // Fill only the visible LEDs (indices 1 to 29)
    fill_solid(lidar_leds + 1, LIDAR_NUM_LEDS, active_base_color);
    lidar_leds[0] = CRGB::Black; // Ensure dummy is off
    FastLED.show();
}

void led_set_yellow() {
    active_base_color = CRGB::Yellow;
    // Fill only the visible LEDs (indices 1 to 29)
    fill_solid(lidar_leds + 1, LIDAR_NUM_LEDS, active_base_color);
    lidar_leds[0] = CRGB::Black; // Ensure dummy is off
    FastLED.show();
}

void led_set_reset() {
    active_base_color = CRGB::White;
    is_pulsing = false;
    FastLED.setBrightness(180); // Reset to default brightness
    // Fill only the visible LEDs (indices 1 to 29)
    fill_solid(lidar_leds + 1, LIDAR_NUM_LEDS, active_base_color);
    lidar_leds[0] = CRGB::Black; // Ensure dummy is off
    FastLED.show();
}

void led_update_from_lidar(int* values, int count) {
    // Handle Pulse Animation (Breathing Effect)
    if (is_pulsing) {
        // beatsin8(BPM, min_brightness, max_brightness)
        // 30 BPM, oscillating between brightness 40 and 160
        FastLED.setBrightness(beatsin8(30, 40, 220));
    } else {
        FastLED.setBrightness(180);
    }

    // Generate a blink state (On/Off every 150ms)
    bool is_blink_on = (millis() / 150) % 2 == 0;

    for (int i = 0; i < count && i < LIDAR_NUM_LEDS; i++) {
        // Fix mirroring: Map index to reverse direction
        // Rotate 180 degrees: Add offset of half the strip length
        int index_reversed = (LIDAR_NUM_LEDS - i) % LIDAR_NUM_LEDS;
        int led_index = (index_reversed + (LIDAR_NUM_LEDS / 2)) % LIDAR_NUM_LEDS;
        int physical_index = led_index + 1; // Shift +1 for sacrificial pixel

        if (values[i] == 0) {
            // Blink RED if mostly blocked (value 0)
            lidar_leds[physical_index] = is_blink_on ? CRGB::Red : CRGB::Black;
        } else {
            // Normal fade behavior for non-zero values
            // Value 1 (Danger) -> Red, Value 100 (Safe) -> Base Color
            int safe_percentage = constrain(values[i], 0, 100);
            uint8_t blend_amount = map(safe_percentage, 0, 100, 0, 255);
            
            lidar_leds[physical_index] = blend(CRGB::Red, active_base_color, blend_amount);
        }
    }
    lidar_leds[0] = CRGB::Black; // Ensure dummy is off
    FastLED.show();
}