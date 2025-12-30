#include <lvgl.h>

// #include <examples/lv_examples.h>
#include <demos/lv_demos.h>
#include "esp_lcd_touch_axs5106l.h"
#define Touch_I2C_SDA 18
#define Touch_I2C_SCL 19
#define Touch_RST 20
#define Touch_INT 21
#define ROTATION 3
#define GFX_BACKLIGHT_PWM 180 // PWM value for backlight (0-255, 180 = ~70% brightness)

// #define DIRECT_RENDER_MODE // Uncomment to enable full frame buffer

#include <Arduino_GFX_Library.h>

#define GFX_BL 23 // default backlight pin, you may replace DF_GFX_BL to actual backlight pin

Arduino_DataBus *bus = new Arduino_HWSPI(15 /* DC */, 14 /* CS */, 1 /* SCK */, 2 /* MOSI */, GFX_NOT_DEFINED /* MISO */);
Arduino_GFX *gfx = new Arduino_ST7789(
    bus, 22 /* RST */, 0 /* rotation */, false /* IPS */,
    172 /* width */, 320 /* height */,
    34 /*col_offset1*/, 0 /*uint8_t row_offset1*/,
    34 /*col_offset2*/, 0 /*row_offset2*/);

uint32_t screenWidth;
uint32_t screenHeight;
uint32_t bufSize;
lv_display_t *disp;
lv_color_t *disp_draw_buf;

#if LV_USE_LOG != 0
void my_print(lv_log_level_t level, const char *buf)
{
  LV_UNUSED(level);
  Serial.println(buf);
  Serial.flush();
}
#endif

uint32_t millis_cb(void)
{
  return millis();
}

void lcd_reg_init(void)
{
  static const uint8_t init_operations[] = {
      BEGIN_WRITE,
      WRITE_COMMAND_8, 0x11, // 2: Out of sleep mode, no args, w/delay
      END_WRITE,
      DELAY, 120,

      BEGIN_WRITE,
      WRITE_C8_D16, 0xDF, 0x98, 0x53,
      WRITE_C8_D8, 0xB2, 0x23,

      WRITE_COMMAND_8, 0xB7,
      WRITE_BYTES, 4,
      0x00, 0x47, 0x00, 0x6F,

      WRITE_COMMAND_8, 0xBB,
      WRITE_BYTES, 6,
      0x1C, 0x1A, 0x55, 0x73, 0x63, 0xF0,

      WRITE_C8_D16, 0xC0, 0x44, 0xA4,
      WRITE_C8_D8, 0xC1, 0x16,

      WRITE_COMMAND_8, 0xC3,
      WRITE_BYTES, 8,
      0x7D, 0x07, 0x14, 0x06, 0xCF, 0x71, 0x72, 0x77,

      WRITE_COMMAND_8, 0xC4,
      WRITE_BYTES, 12,
      0x00, 0x00, 0xA0, 0x79, 0x0B, 0x0A, 0x16, 0x79, 0x0B, 0x0A, 0x16, 0x82,

      WRITE_COMMAND_8, 0xC8,
      WRITE_BYTES, 32,
      0x3F, 0x32, 0x29, 0x29, 0x27, 0x2B, 0x27, 0x28, 0x28, 0x26, 0x25, 0x17, 0x12, 0x0D, 0x04, 0x00, 0x3F, 0x32, 0x29, 0x29, 0x27, 0x2B, 0x27, 0x28, 0x28, 0x26, 0x25, 0x17, 0x12, 0x0D, 0x04, 0x00,

      WRITE_COMMAND_8, 0xD0,
      WRITE_BYTES, 5,
      0x04, 0x06, 0x6B, 0x0F, 0x00,

      WRITE_C8_D16, 0xD7, 0x00, 0x30,
      WRITE_C8_D8, 0xE6, 0x14,
      WRITE_C8_D8, 0xDE, 0x01,

      WRITE_COMMAND_8, 0xB7,
      WRITE_BYTES, 5,
      0x03, 0x13, 0xEF, 0x35, 0x35,

      WRITE_COMMAND_8, 0xC1,
      WRITE_BYTES, 3,
      0x14, 0x15, 0xC0,

      WRITE_C8_D16, 0xC2, 0x06, 0x3A,
      WRITE_C8_D16, 0xC4, 0x72, 0x12,
      WRITE_C8_D8, 0xBE, 0x00,
      WRITE_C8_D8, 0xDE, 0x02,

      WRITE_COMMAND_8, 0xE5,
      WRITE_BYTES, 3,
      0x00, 0x02, 0x00,

      WRITE_COMMAND_8, 0xE5,
      WRITE_BYTES, 3,
      0x01, 0x02, 0x00,

      WRITE_C8_D8, 0xDE, 0x00,
      WRITE_C8_D8, 0x35, 0x00,
      WRITE_C8_D8, 0x3A, 0x05,

      WRITE_COMMAND_8, 0x2A,
      WRITE_BYTES, 4,
      0x00, 0x22, 0x00, 0xCD,

      WRITE_COMMAND_8, 0x2B,
      WRITE_BYTES, 4,
      0x00, 0x00, 0x01, 0x3F,

      WRITE_C8_D8, 0xDE, 0x02,

      WRITE_COMMAND_8, 0xE5,
      WRITE_BYTES, 3,
      0x00, 0x02, 0x00,

      WRITE_C8_D8, 0xDE, 0x00,
      WRITE_C8_D8, 0x36, 0x00,
      WRITE_COMMAND_8, 0x21,
      END_WRITE,

      DELAY, 10,

      BEGIN_WRITE,
      WRITE_COMMAND_8, 0x29, // 5: Main screen turn on, no args, w/delay
      END_WRITE};
  bus->batchOperation(init_operations, sizeof(init_operations));
}

/* LVGL calls it when a rendered image needs to copied to the display*/
void my_disp_flush(lv_display_t *disp, const lv_area_t *area, uint8_t *px_map)
{
#ifndef DIRECT_RENDER_MODE
  uint32_t w = lv_area_get_width(area);
  uint32_t h = lv_area_get_height(area);

  gfx->draw16bitRGBBitmap(area->x1, area->y1, (uint16_t *)px_map, w, h);
#endif // #ifndef DIRECT_RENDER_MODE

  /*Call it to tell LVGL you are ready*/
  lv_disp_flush_ready(disp);
}

void my_touchpad_read(lv_indev_t *indev, lv_indev_data_t *data)
{
  touch_data_t touch_data;
  // uint16_t touchpad_x[1] = { 0 };
  // uint16_t touchpad_y[1] = { 0 };
  uint8_t touchpad_cnt = 0;

  /* Read touch controller data */
  // esp_lcd_touch_read_data(touch_handle);
  bsp_touch_read();
  /* Get coordinates */
  // bool touchpad_pressed = esp_lcd_touch_get_coordinates(touch_handle, touchpad_x, touchpad_y, NULL, &touchpad_cnt, 1);
  bool touchpad_pressed = bsp_touch_get_coordinates(&touch_data);

  if (touchpad_pressed)
  {
    data->point.x = touch_data.coords[0].x;
    data->point.y = touch_data.coords[0].y;
    data->state = LV_INDEV_STATE_PRESSED;
    // printf("x:%03d, y:%03d\r\n", touchpad_x[0], touchpad_y[0]);
  }
  else
  {
    data->state = LV_INDEV_STATE_RELEASED;
  }
}

void lvgl_init_touch()
{
  lv_indev_t *indev = lv_indev_create();
  lv_indev_set_type(indev, LV_INDEV_TYPE_POINTER);
  lv_indev_set_read_cb(indev, my_touchpad_read);
  Wire.begin(Touch_I2C_SDA, Touch_I2C_SCL);
  bsp_touch_init(&Wire, Touch_RST, Touch_INT, gfx->getRotation(), gfx->width(), gfx->height());
}
void setup()
{
#ifdef DEV_DEVICE_INIT
  DEV_DEVICE_INIT();
#endif

  Serial.begin(115200);
  // Serial.setDebugOutput(true);
  // while(!Serial);
  Serial.println("Arduino_GFX LVGL_Arduino_v9 example ");
  String LVGL_Arduino = String('V') + lv_version_major() + "." + lv_version_minor() + "." + lv_version_patch();
  Serial.println(LVGL_Arduino);

  // Init Display
  if (!gfx->begin())
  {
    Serial.println("gfx->begin() failed!");
  }
  lcd_reg_init();
  gfx->setRotation(ROTATION);

#ifdef GFX_BL
  pinMode(GFX_BL, OUTPUT);
  analogWrite(GFX_BL, GFX_BACKLIGHT_PWM);
#endif

  lv_init();

  /*Set a tick source so that LVGL will know how much time elapsed. */
  lv_tick_set_cb(millis_cb);

  /* register print function for debugging */
#if LV_USE_LOG != 0
  lv_log_register_print_cb(my_print);
#endif

  screenWidth = gfx->width();
  screenHeight = gfx->height();

#ifdef DIRECT_RENDER_MODE
  bufSize = screenWidth * screenHeight;
#else
  bufSize = screenWidth * 40;
#endif

#ifdef ESP32
#if defined(DIRECT_RENDER_MODE) && (defined(CANVAS) || defined(RGB_PANEL) || defined(DSI_PANEL))
  disp_draw_buf = (lv_color_t *)gfx->getFramebuffer();
#else  // !(defined(DIRECT_RENDER_MODE) && (defined(CANVAS) || defined(RGB_PANEL) || defined(DSI_PANEL)))
  disp_draw_buf = (lv_color_t *)heap_caps_malloc(bufSize * 2, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
  if (!disp_draw_buf)
  {
    // remove MALLOC_CAP_INTERNAL flag try again
    disp_draw_buf = (lv_color_t *)heap_caps_malloc(bufSize * 2, MALLOC_CAP_8BIT);
  }
#endif // !(defined(DIRECT_RENDER_MODE) && (defined(CANVAS) || defined(RGB_PANEL) || defined(DSI_PANEL)))
#else  // !ESP32
  Serial.println("LVGL disp_draw_buf heap_caps_malloc failed! malloc again...");
  disp_draw_buf = (lv_color_t *)malloc(bufSize * 2);
#endif // !ESP32
  if (!disp_draw_buf)
  {
    Serial.println("LVGL disp_draw_buf allocate failed!");
  }
  else
  {
    disp = lv_display_create(screenWidth, screenHeight);
    lv_display_set_flush_cb(disp, my_disp_flush);
#ifdef DIRECT_RENDER_MODE
    lv_display_set_buffers(disp, disp_draw_buf, NULL, bufSize * 2, LV_DISPLAY_RENDER_MODE_DIRECT);
#else
    lv_display_set_buffers(disp, disp_draw_buf, NULL, bufSize * 2, LV_DISPLAY_RENDER_MODE_PARTIAL);
#endif

    lvgl_init_touch();
    /* Option 1: Create a simple label
     * ---------------------
     */
    lv_obj_t *label = lv_label_create(lv_scr_act());
    lv_label_set_text(label, "Hello Arduino, I'm LVGL!(V" GFX_STR(LVGL_VERSION_MAJOR) "." GFX_STR(LVGL_VERSION_MINOR) "." GFX_STR(LVGL_VERSION_PATCH) ")");
    lv_obj_align(label, LV_ALIGN_CENTER, 0, 0);

    /* Option 2: Try an example. See all the examples
     *  - Online: https://docs.lvgl.io/master.pio/libdeps/esp32-c6-devkitc-1/lvgl/demos/examples.html
     *  - Source codes: https://github.com/lvgl/lvgl/tree/master/examples
     * ----------------------------------------------------------------
     */
    // lv_example_btn_1();

    /* Option 3: Or try out a demo. Don't forget to enable the demos in lv_conf.h. E.g. LV_USE_DEMOS_WIDGETS
     * -------------------------------------------------------------------------------------------
     */
    lv_demo_widgets();
    // lv_demo_benchmark();
    // lv_demo_keypad_encoder();
    // lv_demo_music();
    // lv_demo_stress();
  }

  Serial.println("Setup done");
}

void loop()
{
  lv_task_handler(); /* let the GUI do its work */

#ifdef DIRECT_RENDER_MODE
#if defined(CANVAS) || defined(RGB_PANEL) || defined(DSI_PANEL)
  gfx->flush();
#else  // !(defined(CANVAS) || defined(RGB_PANEL) || defined(DSI_PANEL))
  gfx->draw16bitRGBBitmap(0, 0, (uint16_t *)disp_draw_buf, screenWidth, screenHeight);
#endif // !(defined(CANVAS) || defined(RGB_PANEL) || defined(DSI_PANEL))
#else  // !DIRECT_RENDER_MODE
#ifdef CANVAS
  gfx->flush();
#endif
#endif // !DIRECT_RENDER_MODE
  delay(5);
}
