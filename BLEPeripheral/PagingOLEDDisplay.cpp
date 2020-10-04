#include "PagingOLEDDisplay.h"
#include <stdarg.h>

#include "Debug.h"

static PagingOLEDDisplay *singletonDisplay;

// Switch pages on the display when we receive an interrupt
void pageISR() {
  static unsigned long lastInterruptTime = 0;
  unsigned long interruptTime = millis();
  if (interruptTime - lastInterruptTime > 150) {
    singletonDisplay->currentPage_ =
        (singletonDisplay->currentPage_ + 1) % singletonDisplay->nbrOfPages_;
  }
  lastInterruptTime = interruptTime;
}

PagingOLEDDisplay::PagingOLEDDisplay(unsigned int width, unsigned int height,
                                     unsigned int nbrOfLines,
                                     unsigned int pagingPin)
    : displayHeight_(height), displayWidth_(width),
      display_((int)width, (int)height), nbrOfLines_(nbrOfLines),
      pagingPin_(pagingPin), currentPage_(0) {
  singletonDisplay = this;
}

boolean PagingOLEDDisplay::begin() {
  bool retStatus = true;

  pinMode(pagingPin_, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(pagingPin_), pageISR, FALLING);

  retStatus &= display_.begin(SSD1306_SWITCHCAPVCC, 0x3C);

  // Get the size of "m"
  int16_t x1, y1;
  uint16_t mWidth, mHeight;
  display_.getTextBounds("M", 0, 0, &x1, &y1, &mWidth, &mHeight);
  lineHeight_ = mHeight + 2;

  // Nbr of pages is based on a line height of capital M plus 2 pixels
  linesPerPage_ = displayHeight_ / lineHeight_;

  // round up the nbr of lines to a full page then add 1 page extra for a blank
  // page;
  nbrOfPages_ = ceil((float)nbrOfLines_ / (float)linesPerPage_) + 1;

  for (int i = 0; i < nbrOfLines_; i++)
    lineBuffer_.push_back("");

  return retStatus;
}

void PagingOLEDDisplay::displayPage(unsigned int page) {
  currentPage_ = page;
  display_.clearDisplay();
  if (currentPage_ < (nbrOfPages_ - 1)) {
    display_.setTextColor(SSD1306_WHITE);
    for (uint8_t i = 0; i < linesPerPage_; i++) {
      uint8_t line = page * linesPerPage_ + i;
      if (line < nbrOfLines_) {
        display_.setCursor(0, i * lineHeight_);
        display_.print(lineBuffer_[page * linesPerPage_ + i].c_str());
      }
    }
  }
  display_.display();
}

void PagingOLEDDisplay::displayCurrentPage() { displayPage(currentPage_); }

void PagingOLEDDisplay::printf(unsigned int line, const char *fmt, ...) {
  char buff[64];
  va_list argptr;
  va_start(argptr, fmt);
  vsnprintf(buff, 64, fmt, argptr);
  va_end(argptr);
  lineBuffer_[line % nbrOfLines_] = buff;
}

void PagingOLEDDisplay::clear() {
  for (unsigned int i = 0; i < nbrOfLines_; i++)
    lineBuffer_[i] = "";
  display_.clearDisplay();
}
