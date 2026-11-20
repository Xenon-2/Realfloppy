#include <UTFT.h>
#include <Wire.h>

extern uint8_t SmallFont[];
extern uint8_t BigFont[];
extern uint8_t SevenSegNumFont[];

UTFT myGLCD(ITDB32S, 38, 39, 40, 41);

#define T_CLK  6
#define T_CS   5
#define T_DIN  4
#define T_DOUT 3
#define T_IRQ  2

#define PICO_I2C_ADDR 0x42

// Palette RGB565
#define C_BG        0x10A2
#define C_PANEL     0x2124
#define C_ACCENT    0x07FF
#define C_GREEN     0x07E0
#define C_AMBER     0xFD20
#define C_RED       0xF800
#define C_WHITE     0xFFFF
#define C_GRAY      0x8410

enum ScreenState {
  SCREEN_STANDBY,
  SCREEN_DASHBOARD,
  SCREEN_BROWSER,
  SCREEN_MAP,
  SCREEN_PROTO,
  SCREEN_CONFIRM_WRITE,
  SCREEN_CONFIRM_DEL
};
ScreenState currentScreen = SCREEN_STANDBY;

String currentProtocol = "DOS 720K";
String currentFile     = "(None)";
int currentTrack       = 0;
int maxTracks          = 80;
int maxSides           = 2;
int currentSide        = 0;
String statusMsg       = "READY";
String i2cBuffer       = "";

// Hardware/PC (Default: 3.5" and Greaseweazle to match Pico boot state)
bool isDrive525   = false; // false = Drive B (3.5"), true = Drive A (5.25" 360K)
bool isRealFloppy = false; // false = Greaseweazle USB, true = RealFloppy UART1

String fileList[6];
int fileCountOnPage   = 0;
int totalFilesOnSD    = 0;
int currentFileOffset = 0;

const char* protocols[6] = {
  "DOS 720K", "DOS 360K",
  "DOS 180K", "AMIGA ADF",
  "ATARI ST", "RAW FLUX"
};

// ==========================================
// TOUCH XPT2046
// ==========================================
uint16_t readXPT2046(uint8_t cmd) {
  uint16_t data = 0;
  digitalWrite(T_CS, LOW);
  for (int i = 7; i >= 0; i--) {
    digitalWrite(T_DIN, (cmd >> i) & 1);
    digitalWrite(T_CLK, HIGH); delayMicroseconds(2);
    digitalWrite(T_CLK, LOW);  delayMicroseconds(2);
  }
  digitalWrite(T_CLK, HIGH); delayMicroseconds(2);
  digitalWrite(T_CLK, LOW);  delayMicroseconds(2);
  for (int i = 11; i >= 0; i--) {
    digitalWrite(T_CLK, HIGH); delayMicroseconds(2);
    if (digitalRead(T_DOUT)) data |= (1 << i);
    digitalWrite(T_CLK, LOW);  delayMicroseconds(2);
  }
  for (int i = 0; i < 4; i++) {
    digitalWrite(T_CLK, HIGH); delayMicroseconds(2);
    digitalWrite(T_CLK, LOW);  delayMicroseconds(2);
  }
  digitalWrite(T_CS, HIGH);
  return data;
}

bool getTouchPoint(int &px, int &py) {
  if (digitalRead(T_IRQ) != LOW) return false;
  delay(12);
  uint32_t sx1 = 0, sy1 = 0, sx2 = 0, sy2 = 0;
  for (int i = 0; i < 4; i++) { sx1 += readXPT2046(0x90); sy1 += readXPT2046(0xD0); }
  delay(5);
  for (int i = 0; i < 4; i++) { sx2 += readXPT2046(0x90); sy2 += readXPT2046(0xD0); }
  int rx1 = sx1 / 4, ry1 = sy1 / 4, rx2 = sx2 / 4, ry2 = sy2 / 4;
  if (abs(rx1 - rx2) > 120 || abs(ry1 - ry2) > 120) return false;
  int rawX = (rx1 + rx2) / 2, rawY = (ry1 + ry2) / 2;
  if (rawX <= 50 || rawX >= 4050 || rawY <= 50 || rawY >= 4050) return false;
  px = constrain(map(rawX, 580, 3800, 15, 304), 0, 319);
  py = constrain(map(rawY, 395, 3560, 15, 224), 0, 239);
  return true;
}

// ==========================================
// I2C PICO (0x42)
// ==========================================
void sendPicoCommand(String cmd) {
  cmd += "\n";
  for (unsigned int i = 0; i < cmd.length(); i += 28) {
    Wire.beginTransmission(PICO_I2C_ADDR);
    String chunk = cmd.substring(i, min((unsigned int)cmd.length(), i + 28));
    Wire.write((const uint8_t*)chunk.c_str(), chunk.length());
    Wire.endTransmission();
  }
}

void readPicoI2C() {
  static uint32_t lastPollMs = 0;
  if (millis() - lastPollMs < 35) return;
  lastPollMs = millis();

  for (int p = 0; p < 12; p++) {
    uint8_t n = Wire.requestFrom(PICO_I2C_ADDR, 30);
    if (n == 0) break;
    bool got = false;
    while (Wire.available()) {
      char c = Wire.read();
      if (c == 0x00) break;
      got = true;
      if (c == '\n') {
        parsePicoCommand(i2cBuffer);
        i2cBuffer = "";
      } else if (c != '\r') {
        i2cBuffer += c;
      }
    }
    if (!got) break;
  }
}

void setup() {
  // Serial.begin(115200);
  Wire.begin();

  maxTracks       = isDrive525 ? 40 : 80;
  currentProtocol = isDrive525 ? "DOS 360K" : "DOS 720K";
  maxSides        = 2;

  pinMode(T_CLK, OUTPUT);
  pinMode(T_CS, OUTPUT);
  pinMode(T_DIN, OUTPUT);
  pinMode(T_DOUT, INPUT);
  pinMode(T_IRQ, INPUT);
  digitalWrite(T_CS, HIGH);
  digitalWrite(T_CLK, LOW);

  myGLCD.InitLCD(LANDSCAPE);
  drawStandbyScreen();
}

void loop() {
  readPicoI2C();

  static uint32_t lastTouchMs = 0;
  int x, y;
  if (getTouchPoint(x, y)) {
    if (millis() - lastTouchMs > 220) {
      lastTouchMs = millis();
      handleTouch(x, y);
    }
  }
}

// ==========================================
// UI
// ==========================================
void drawStandbyScreen() {
  currentScreen = SCREEN_STANDBY;
  myGLCD.fillScr(C_BG);
  drawHeader("REALFLOPPY", "PC MODE");

  myGLCD.setBackColor(C_BG);
  myGLCD.setColor(C_GRAY);
  myGLCD.setFont(SmallFont);
  myGLCD.print("ACTIVE CONFIGURATION (TOUCH TO CHANGE):", CENTER, 42);

  // Left button: Drive selection (3.5" vs 5.25")
  String drvLabel = isDrive525 ? "DRV A: 5.25\"" : "DRV B: 3.5\"";
  word drvColor   = isDrive525 ? C_AMBER : C_ACCENT;
  drawButton(12, 62, 155, 134, drvLabel, drvColor, false);

  // Right button: PC Protocol selection (Greaseweazle vs RealFloppy)
  String modeLabel = isRealFloppy ? "REALFLOPPY" : "GREASEWEAZLE";
  word modeColor   = isRealFloppy ? C_AMBER : C_ACCENT;
  drawButton(164, 62, 307, 134, modeLabel, modeColor, false);

  // Bottom button: Enter stand-alone SD Card mode
  drawButton(12, 152, 307, 224, "FLOPPY STATION MODE", C_GREEN, false);
}

void drawDashboard() {
  currentScreen = SCREEN_DASHBOARD;
  myGLCD.fillScr(C_BG);
  drawHeader("REALFLOPPY", currentProtocol);

  myGLCD.setColor(C_PANEL);
  myGLCD.fillRoundRect(8, 36, 311, 74);
  myGLCD.setColor(C_GRAY);
  myGLCD.drawRoundRect(8, 36, 311, 74);
  myGLCD.setBackColor(C_PANEL);
  myGLCD.setColor(C_ACCENT);
  myGLCD.setFont(SmallFont);
  myGLCD.print("Selected image:", 16, 41);
  updateFileName(currentFile);

  myGLCD.setColor(C_PANEL);
  myGLCD.fillRoundRect(8, 82, 311, 165);
  myGLCD.setColor(C_GRAY);
  myGLCD.drawRoundRect(8, 82, 311, 165);
  myGLCD.setBackColor(C_PANEL);
  myGLCD.setColor(C_GRAY);
  myGLCD.setFont(SmallFont);
  myGLCD.print("Track", 25, 88);
  myGLCD.print("Side", 130, 88);
  myGLCD.print("STATE", 190, 88);

  updateTrackDisplay(currentTrack, currentSide);
  updateStatus(statusMsg, C_GREEN);

  // 4 Bottom buttons: FILE SD | FLOPPY | PROTO | EXIT
  drawButton(8,   175, 84,  232, "FILE SD", C_ACCENT, false);
  drawButton(89,  175, 165, 232, "FLOPPY",  C_AMBER,  false);
  drawButton(170, 175, 246, 232, "PROTO",   C_WHITE,  false);
  drawButton(251, 175, 311, 232, "EXIT",    C_RED,    false);
}

void drawBrowser() {
  currentScreen = SCREEN_BROWSER;
  myGLCD.fillScr(C_BG);
  drawHeader("SD CARD", String(totalFilesOnSD) + " FILES");

  for (int i = 0; i < 6; i++) {
    int y1 = 36 + (i * 33);
    int y2 = y1 + 29;
    myGLCD.setColor(C_PANEL);
    myGLCD.fillRoundRect(8, y1, 235, y2);
    if (i < fileCountOnPage) {
      bool isDir = fileList[i].endsWith("/");
      bool isSelected = (fileList[i] == currentFile);
      myGLCD.setColor(isSelected ? C_GREEN : (isDir ? C_AMBER : C_ACCENT));
      myGLCD.drawRoundRect(8, y1, 235, y2);
      myGLCD.setBackColor(C_PANEL);
      myGLCD.setColor(isSelected ? C_GREEN : (isDir ? C_AMBER : C_WHITE));
      myGLCD.setFont(SmallFont);
      String name = fileList[i];
      if (name.length() > 26) name = name.substring(0, 25) + "~";
      myGLCD.print(name, 14, y1 + 9);
    } else {
      myGLCD.setColor(C_GRAY);
      myGLCD.drawRoundRect(8, y1, 235, y2);
    }
  }

  // 4 Navigation/action buttons
  drawButton(242, 36,  311, 81,  "UP",     C_ACCENT, false);
  drawButton(242, 86,  311, 131, "DOWN",   C_ACCENT, false);
  drawButton(242, 136, 311, 181, "DELETE", C_AMBER,  false);
  drawButton(242, 186, 311, 231, "EXIT",   C_RED,    false);
}

void drawConfirmDelete() {
  currentScreen = SCREEN_CONFIRM_DEL;
  myGLCD.setColor(C_PANEL);
  myGLCD.fillRoundRect(20, 45, 300, 195);
  myGLCD.setColor(C_RED);
  myGLCD.drawRoundRect(20, 45, 300, 195);
  myGLCD.drawRoundRect(21, 46, 299, 194);

  myGLCD.setBackColor(C_PANEL);
  myGLCD.setColor(C_RED);
  myGLCD.setFont(BigFont);
  myGLCD.print("DELETE FILE?", CENTER, 58);

  myGLCD.setColor(C_WHITE);
  myGLCD.setFont(SmallFont);
  String disp = currentFile;
  if (disp.length() > 28) disp = disp.substring(0, 27) + "~";
  myGLCD.print(disp, CENTER, 94);

  drawButton(40,  135, 150, 180, "YES, DELETE", C_RED,   false);
  drawButton(170, 135, 280, 180, "CANCEL",      C_WHITE, false);
}

// ==========================================
// X-COPY STYLE: SIDE 0 | SIDE 1
// ==========================================
void drawSectorMap() {
  currentScreen = SCREEN_MAP;
  myGLCD.fillScr(C_BG);
  drawHeader("FLOPPY OPS", currentProtocol);

  myGLCD.setFont(SmallFont);
  myGLCD.setBackColor(C_BG);

  // Rows (4 for 40 trk, 8 for 80 trk)
  int numRows = (maxTracks <= 40) ? 4 : 8;

  // SIDE 0 and SIDE 1 labels
  myGLCD.setColor(C_ACCENT);
  myGLCD.print("SIDE 0", 54, 34);
  if (maxSides > 1) {
    myGLCD.print("SIDE 1", 206, 34);
  }

  // Grid origin
  int gridX[2]  = { 26, 178 };
  int labelX[2] = { 14, 166 };
  int startY    = 62;
  int cellSize  = 12; // 12x12 px

  for (int side = 0; side < maxSides; side++) {
    // Columns 0..9 on top
    myGLCD.setColor(C_WHITE);
    for (int col = 0; col < 10; col++) {
      char cStr[2] = { (char)('0' + col), '\0' };
      myGLCD.print(cStr, gridX[side] + (col * cellSize) + 2, 48);
    }

    // Rows (0..3 or 0..7) on left
    for (int row = 0; row < numRows; row++) {
      myGLCD.setColor(C_WHITE);
      char rStr[2] = { (char)('0' + row), '\0' };
      myGLCD.print(rStr, labelX[side], startY + (row * cellSize));

      for (int col = 0; col < 10; col++) {
        int x = gridX[side] + (col * cellSize);
        int y = startY + (row * cellSize);
        myGLCD.setColor(C_GRAY);
        myGLCD.drawRect(x, y, x + cellSize, y + cellSize);
        myGLCD.setColor(C_PANEL);
        myGLCD.drawCircle(x + 6, y + 6, 4);
      }
    }
  }

  // 3 Bottom buttons
  drawButton(8,   178, 106, 232, "READ->SD", C_GREEN, false);
  drawButton(112, 178, 208, 232, "WRITE",    C_AMBER, false);
  drawButton(214, 178, 311, 232, "EXIT",     C_RED,   false);
}

void updateSectorCell(int trk, int side, int state) {
  if (currentScreen != SCREEN_MAP) return;
  if (trk < 0 || trk >= maxTracks || side < 0 || side >= maxSides) return;

  int gridX[2] = { 26, 178 };
  int startY   = 62;
  int cellSize = 12;

  int col = trk % 10; // Track units (0..9)
  int row = trk / 10; // Track tens (0..3 or 0..7)

  int x = gridX[side] + (col * cellSize);
  int y = startY + (row * cellSize);

  word color = (state == 0) ? C_GREEN : (state == 1 ? C_AMBER : C_RED);
  myGLCD.setColor(color);
  myGLCD.drawCircle(x + 6, y + 6, 4);
}

void drawProtocolSelector() {
  currentScreen = SCREEN_PROTO;
  myGLCD.fillScr(C_BG);
  drawHeader("SELECT PROTO", "SYSTEM");

  for (int i = 0; i < 6; i++) {
    int x1 = 12 + (i % 2) * 152;
    int y1 = 42 + (i / 2) * 62;
    bool active = (currentProtocol == String(protocols[i]));
    drawButton(x1, y1, x1 + 142, y1 + 52, protocols[i], active ? C_GREEN : C_ACCENT, active);
  }
}

void drawConfirmWrite() {
  currentScreen = SCREEN_CONFIRM_WRITE;
  myGLCD.setColor(C_PANEL);
  myGLCD.fillRoundRect(25, 45, 295, 195);
  myGLCD.setColor(C_RED);
  myGLCD.drawRoundRect(25, 45, 295, 195);
  myGLCD.drawRoundRect(26, 46, 294, 194);

  myGLCD.setBackColor(C_PANEL);
  myGLCD.setColor(C_RED);
  myGLCD.setFont(BigFont);
  myGLCD.print("ATTENTION!", CENTER, 60);

  myGLCD.setColor(C_WHITE);
  myGLCD.setFont(SmallFont);
  myGLCD.print("Floppy will be", CENTER, 90);
  myGLCD.print("overwritten", CENTER, 106);

  drawButton(40,  135, 150, 180, "CONFIRM", C_RED,   false);
  drawButton(170, 135, 280, 180, "CANCEL",  C_WHITE, false);
}

// ==========================================
// HELPERS
// ==========================================
void drawHeader(String title, String badge) {
  myGLCD.setColor(C_PANEL);
  myGLCD.fillRect(0, 0, 319, 28);
  myGLCD.setColor(C_ACCENT);
  myGLCD.drawLine(0, 29, 319, 29);
  myGLCD.setBackColor(C_PANEL);
  myGLCD.setColor(C_WHITE);
  myGLCD.setFont(BigFont);
  myGLCD.print(title, 8, 6);

  myGLCD.setColor(C_ACCENT);
  myGLCD.fillRoundRect(212, 4, 315, 24);
  myGLCD.setBackColor(C_ACCENT);
  myGLCD.setColor(C_BG);
  myGLCD.setFont(SmallFont);
  myGLCD.print(badge, 218, 9);
}

void drawButton(int x1, int y1, int x2, int y2, String label, word color, bool pressed) {
  myGLCD.setColor(pressed ? color : C_PANEL);
  myGLCD.fillRoundRect(x1, y1, x2, y2);
  myGLCD.setColor(color);
  myGLCD.drawRoundRect(x1, y1, x2, y2);
  myGLCD.drawRoundRect(x1 + 1, y1 + 1, x2 - 1, y2 - 1);
  myGLCD.setBackColor(pressed ? color : C_PANEL);
  myGLCD.setColor(pressed ? C_BG : color);
  myGLCD.setFont(SmallFont);
  int textX = x1 + ((x2 - x1) - (label.length() * 8)) / 2;
  int textY = y1 + ((y2 - y1) - 12) / 2;
  myGLCD.print(label, textX, textY);
}

void flashButton(int x1, int y1, int x2, int y2, String label, word color) {
  drawButton(x1, y1, x2, y2, label, color, true);
  delay(80);
  drawButton(x1, y1, x2, y2, label, color, false);
}

void updateFileName(String fname) {
  currentFile = fname;
  if (currentScreen != SCREEN_DASHBOARD) return;
  myGLCD.setColor(C_PANEL);
  myGLCD.fillRect(14, 55, 305, 70);
  myGLCD.setBackColor(C_PANEL);
  myGLCD.setColor(C_WHITE);
  myGLCD.setFont(BigFont);
  if (fname.length() > 18) fname = fname.substring(0, 17) + "~";
  myGLCD.print(fname, 16, 55);
}

void updateTrackDisplay(int trk, int side) {
  currentTrack = trk;
  currentSide = side;
  if (currentScreen != SCREEN_DASHBOARD) return;
  myGLCD.setBackColor(C_PANEL);
  myGLCD.setColor(C_GREEN);
  myGLCD.setFont(SevenSegNumFont);
  char buf[8];
  sprintf(buf, "%02d", trk);
  myGLCD.print(buf, 22, 105);
  sprintf(buf, "%d", side);
  myGLCD.print(buf, 132, 105);
  int barWidth = map(constrain(trk, 0, maxTracks), 0, maxTracks, 0, 286);
  myGLCD.setColor(C_BG);
  myGLCD.fillRect(16, 152, 302, 159);
  myGLCD.setColor(C_ACCENT);
  myGLCD.fillRect(16, 152, 16 + barWidth, 159);
}

void updateStatus(String msg, word color) {
  statusMsg = msg;
  if (currentScreen != SCREEN_DASHBOARD) return;
  myGLCD.setColor(C_PANEL);
  myGLCD.fillRect(185, 105, 305, 145);
  myGLCD.setBackColor(C_PANEL);
  myGLCD.setColor(color);
  myGLCD.setFont(SmallFont);
  myGLCD.print(msg, 188, 118);
}

// ==========================================
// TOUCH HANDLER
// ==========================================
void handleTouch(int x, int y) {
  if (currentScreen == SCREEN_STANDBY) {
    // 1. Left Button -> Toggle Drive (3.5" <-> 5.25")
    if (x >= 12 && x <= 155 && y >= 62 && y <= 134) {
      isDrive525 = !isDrive525;
      if (isDrive525) {
        maxTracks = 40;
        maxSides  = 2;
        currentProtocol = "DOS 360K";
        sendPicoCommand("CMD:SET_DRIVE:A");
      } else {
        maxTracks = 80;
        maxSides  = 2;
        currentProtocol = "DOS 720K";
        sendPicoCommand("CMD:SET_DRIVE:B");
      }
      delay(10);
      sendPicoCommand("CMD:SET_PROTO:" + currentProtocol);
      drawStandbyScreen();
    }
    // 2. Right Button -> Toggle PC Protocol (GW <-> RF)
    else if (x >= 164 && x <= 307 && y >= 62 && y <= 134) {
      isRealFloppy = !isRealFloppy;
      sendPicoCommand(isRealFloppy ? "CMD:SET_MODE:RF" : "CMD:SET_MODE:GW");
      drawStandbyScreen();
    }
    // 3. Green Button -> Enter Floppy Station Mode
    else if (x >= 12 && x <= 307 && y >= 152 && y <= 224) {
      flashButton(12, 152, 307, 224, "FLOPPY STATION MODE", C_GREEN);
      sendPicoCommand("CMD:ENTER_FS");
      drawDashboard();
    }
  }
  else if (currentScreen == SCREEN_DASHBOARD) {
    if (y >= 172 && y <= 238) {
      if (x >= 8 && x <= 84) {
        flashButton(8, 175, 84, 232, "FILES SD", C_ACCENT);
        currentFileOffset = 0;
        sendPicoCommand("CMD:GET_FILES:0");
        drawBrowser();
      } 
      else if (x >= 89 && x <= 165) {
        flashButton(89, 175, 165, 232, "FLOPPY", C_AMBER);
        drawSectorMap();
      } 
      else if (x >= 170 && x <= 246) {
        flashButton(170, 175, 246, 232, "PROTO", C_WHITE);
        drawProtocolSelector();
      } 
      else if (x >= 251 && x <= 311) {
        flashButton(251, 175, 311, 232, "EXIT", C_RED);
        sendPicoCommand("CMD:EXIT_FS");
        drawStandbyScreen();
      }
    }
  }
  else if (currentScreen == SCREEN_BROWSER) {
    if (x >= 8 && x <= 235 && y >= 36 && y <= 230) {
      int idx = (y - 36) / 33;
      if (idx >= 0 && idx < fileCountOnPage) {
        sendPicoCommand("CMD:SELECT:" + fileList[idx]);
        if (!fileList[idx].endsWith("/")) {
          currentFile = fileList[idx];
          drawBrowser(); 
        }
      }
    } 
    else if (x >= 242 && x <= 311) {
      if (y >= 36 && y <= 81) {
        flashButton(242, 36, 311, 81, "UP", C_ACCENT);
        if (currentFileOffset >= 6) {
          currentFileOffset -= 6;
          sendPicoCommand("CMD:GET_FILES:" + String(currentFileOffset));
        }
      } 
      else if (y >= 86 && y <= 131) {
        flashButton(242, 86, 311, 131, "DOWN", C_ACCENT);
        if ((currentFileOffset + 6) < totalFilesOnSD) {
          currentFileOffset += 6;
          sendPicoCommand("CMD:GET_FILES:" + String(currentFileOffset));
        }
      } 
      else if (y >= 136 && y <= 181) {
        flashButton(242, 136, 311, 181, "DELETE", C_AMBER);
        if (currentFile.length() > 0 && !currentFile.endsWith("/")) {
          drawConfirmDelete();
        }
      } 
      else if (y >= 186 && y <= 235) {
        flashButton(242, 186, 311, 231, "EXIT", C_RED);
        drawDashboard();
      }
    }
  }
  else if (currentScreen == SCREEN_CONFIRM_DEL) {
    if (y >= 135 && y <= 180) {
      if (x >= 40 && x <= 150) {
        sendPicoCommand("CMD:DEL:" + currentFile);
        currentFile = "(None)";
        currentFileOffset = 0;
        delay(50);
        sendPicoCommand("CMD:GET_FILES:0");
        drawBrowser();
      } 
      else if (x >= 170 && x <= 280) {
        drawBrowser();
      }
    }
  }
  else if (currentScreen == SCREEN_MAP) {
    if (y >= 178 && y <= 232) {
      if (x >= 8 && x <= 106) {
        flashButton(8, 178, 106, 232, "READ->SD", C_GREEN);
        sendPicoCommand("CMD:DUMP");
      }
      else if (x >= 112 && x <= 208) {
        flashButton(112, 178, 208, 232, "WRITE", C_AMBER);
        drawConfirmWrite();
      } 
      else if (x >= 214 && x <= 311) {
        flashButton(214, 178, 311, 232, "EXIT", C_RED);
        drawDashboard();
      }
    }
  }
  else if (currentScreen == SCREEN_CONFIRM_WRITE) {
    if (y >= 135 && y <= 180) {
      if (x >= 40 && x <= 150) {
        sendPicoCommand("CMD:WRITE");
        drawSectorMap();
      } 
      else if (x >= 170 && x <= 280) {
        drawSectorMap();
      }
    }
  }
  else if (currentScreen == SCREEN_PROTO) {
    for (int i = 0; i < 6; i++) {
      int x1 = 12 + (i % 2) * 152;
      int y1 = 42 + (i / 2) * 62;
      if (x >= x1 && x <= x1 + 142 && y >= y1 && y <= y1 + 52) {
        currentProtocol = String(protocols[i]);
        if (currentProtocol == "DOS 180K") {
          maxTracks = 40;
          maxSides  = 1;
        } else if (currentProtocol == "DOS 360K") {
          maxTracks = 40;
          maxSides  = 2;
        } else {
          maxTracks = 80;
          maxSides  = 2;
        }
        sendPicoCommand("CMD:SET_PROTO:" + currentProtocol);
        drawDashboard();
        break;
      }
    }
  }
}

void parsePicoCommand(String cmd) {
  cmd.trim();
  if (cmd.startsWith("TRK:")) {
    int c1 = cmd.indexOf(':');
    int c2 = cmd.indexOf(':', c1 + 1);
    updateTrackDisplay(cmd.substring(c1 + 1, c2).toInt(), cmd.substring(c2 + 1).toInt());
  } 
  else if (cmd.startsWith("SIZE:")) {
    long fsize = cmd.substring(5).toInt();
    if (fsize == 184320) {
      maxTracks = 40;
      maxSides  = 1;
      currentProtocol = "DOS 180K";
    } else if (fsize == 368640) {
      maxTracks = 40;
      maxSides  = 2;
      currentProtocol = "DOS 360K";
    } else if (fsize == 737280) {
      maxTracks = 80;
      maxSides  = 2;
      currentProtocol = "DOS 720K";
    }
    sendPicoCommand("CMD:SET_PROTO:" + currentProtocol);
    if (currentScreen == SCREEN_DASHBOARD) drawDashboard();
  }
  else if (cmd.startsWith("SEC:")) {
    int c1 = cmd.indexOf(':');
    int c2 = cmd.indexOf(':', c1 + 1);
    int c3 = cmd.indexOf(':', c2 + 1);
    updateSectorCell(cmd.substring(c1 + 1, c2).toInt(), cmd.substring(c2 + 1, c3).toInt(), cmd.substring(c3 + 1).toInt());
  } 
  else if (cmd.startsWith("PAGE:")) {
    int c1 = cmd.indexOf(':');
    int c2 = cmd.indexOf(':', c1 + 1);
    totalFilesOnSD = cmd.substring(c1 + 1, c2).toInt();
    String listStr = cmd.substring(c2 + 1);
    fileCountOnPage = 0;
    int start = 0;
    while (fileCountOnPage < 6 && start < (int)listStr.length()) {
      int pipe = listStr.indexOf('|', start);
      if (pipe == -1) {
        fileList[fileCountOnPage++] = listStr.substring(start);
        break;
      } else {
        fileList[fileCountOnPage++] = listStr.substring(start, pipe);
        start = pipe + 1;
      }
    }
    if (currentScreen == SCREEN_BROWSER) drawBrowser();
  } 
  else if (cmd.startsWith("FILE:")) {
    updateFileName(cmd.substring(5));
  } 
  else if (cmd.startsWith("PROTO:")) {
    currentProtocol = cmd.substring(6);
    if (currentScreen == SCREEN_DASHBOARD) drawDashboard();
  } 
  else if (cmd.startsWith("STAT:")) {
    updateStatus(cmd.substring(5), C_GREEN);
  }
}
