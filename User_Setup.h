// ============================================================
// TFT_eSPI User_Setup.h —— 本项目专用
// 用法：Arduino IDE → 项目文件夹里找到 libraries/TFT_eSPI/
//       把本文件覆盖为 User_Setup.h（或把内容粘贴进去），
//       然后重新编译上传。
//
// 接线（7 脚 SPI 屏）：
//   GND  -> GND
//   VDD  -> 3V3
//   SCK  -> GPIO18
//   SDA  -> GPIO23   （屏上的 SDA = SPI MOSI）
//   RES  -> GPIO14
//   DC   -> GPIO27
//   CS   -> GPIO33
// ============================================================

#define ST7735_DRIVER
#define TFT_WIDTH  128
#define TFT_HEIGHT 160

#define TFT_MISO -1
#define TFT_MOSI 23
#define TFT_SCLK 18
#define TFT_CS   33
#define TFT_DC   27
#define TFT_RST  14

#define LOAD_GLCD
#define LOAD_FONT2

// 颜色反了/画面偏移时按屏幕型号取消注释对应一行：
// #define ST7735_GREENTAB160x80
// #define ST7735_GREENTAB128x128
// #define ST7735_BLACKTAB
// #define TFT_INVERSION_ON

// 如果你的屏是 240x320 的 ILI9341（2.0/2.4 寸），改用下面三行：
// #define ILI9341_DRIVER
// #define TFT_WIDTH  240
// #define TFT_HEIGHT 320
