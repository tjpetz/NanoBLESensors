#include "PagingOLEDDisplay.h"

#define _DEBUG_
#include "Debug.h"

static PagingOLEDDisplay* singletonDisplay;

// Switch pages on the display when we receive an interrupt
void pageISR() {
  static unsigned long lastInterruptTime = 0;
  unsigned long interruptTime = millis();
  if (interruptTime - lastInterruptTime > 150) {
    singletonDisplay->currentPage_ = (singletonDisplay->currentPage_ + 1) % singletonDisplay->maxDisplayPages_;
    singletonDisplay->displayCurrentPage();  
  }
  lastInterruptTime = interruptTime;
}

PagingOLEDDisplay::PagingOLEDDisplay(const int width, const int height, const unsigned int pagingPin) 
    : display_(width, height),
    pagingPin_(pagingPin),
    currentPage_(0),
    maxDisplayPages_(2)
{
    singletonDisplay = this;
}

boolean PagingOLEDDisplay::begin() {
    pinMode(pagingPin_, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(pagingPin_), pageISR, FALLING);
    return display_.begin(SSD1306_SWITCHCAPVCC, 0x3C);
}

void PagingOLEDDisplay::displayPage(unsigned int page) {
    currentPage_ = page;
    DEBUG_PRINTF("Current Page = %d\n", page);
    display_.clearDisplay();
    display_.setTextColor(SSD1306_WHITE);
    display_.setCursor(0, 0); display_.print(displayBuffer_[page * 2]);
    display_.setCursor(0, 25); display_.print(displayBuffer_[page * 2 + 1]);
    display_.display();
}

void PagingOLEDDisplay::displayCurrentPage() {
    displayPage(currentPage_);
}

