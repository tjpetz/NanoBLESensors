#include "PagingOLEDDisplay.h"
#include <stdarg.h>

#define _DEBUG_
#include "Debug.h"

static PagingOLEDDisplay* singletonDisplay;

// Switch pages on the display when we receive an interrupt
void pageISR() {
  static unsigned long lastInterruptTime = 0;
  unsigned long interruptTime = millis();
  if (interruptTime - lastInterruptTime > 150) {
    singletonDisplay->currentPage_ = (singletonDisplay->currentPage_ + 1) % singletonDisplay->nbrOfPages_;
    singletonDisplay->displayCurrentPage();  
  }
  lastInterruptTime = interruptTime;
}

PagingOLEDDisplay::PagingOLEDDisplay(const int width, const int height, const unsigned int nbrOfLines, const unsigned int pagingPin) 
    : display_(width, height),
    nbrOfLines_(nbrOfLines),
    pagingPin_(pagingPin),
    currentPage_(0)
{
    singletonDisplay = this;

    // Nbr of pages is based on a fixed line height of 25 pixels
    // this can be improved in the future by supporting fonts and then calculating the height
    linesPerPage_ = height / 25 + 1;
    // round up the nbr of lines to a full page then add 1 page extra for a blank page;
    nbrOfPages_ = (nbrOfLines_ + nbrOfLines_ % linesPerPage_) / linesPerPage_ + 1;
    
    for (int i = 0; i < nbrOfLines_; i++)
      lineBuffer_.push_back("");
}

boolean PagingOLEDDisplay::begin() {
    pinMode(pagingPin_, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(pagingPin_), pageISR, FALLING);
    return display_.begin(SSD1306_SWITCHCAPVCC, 0x3C);
}

void PagingOLEDDisplay::displayPage(unsigned int page) {
    currentPage_ = page;
    display_.clearDisplay();
    if (currentPage_ < (nbrOfPages_ - 1)) {
        display_.setTextColor(SSD1306_WHITE);
        display_.setCursor(0, 0); display_.print(lineBuffer_[page * 2].c_str());
        display_.setCursor(0, 25); display_.print(lineBuffer_[page * 2 + 1].c_str());
    }
    display_.display();
}

void PagingOLEDDisplay::displayCurrentPage() {
    displayPage(currentPage_);
}

void PagingOLEDDisplay::printf(unsigned int line, const char * fmt, ...) {
    char buff[64];
    va_list argptr;
    va_start(argptr, fmt);
    vsnprintf(buff, 64, fmt, argptr);
    va_end(argptr);
    lineBuffer_[line % nbrOfLines_] = buff;
}

void PagingOLEDDisplay::clear() {
    for (int i = 0; i < nbrOfLines_; i++)
        lineBuffer_[i] = "";
    display_.clearDisplay();
}
