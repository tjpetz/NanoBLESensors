#ifndef PAGINGOLEDDISPLAY_H
#define PAGINGOLEDDISPLAY_H

#define NO_ADAFRUIT_SSD1306_COLOR_COMPATIBILIT//#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

/**
 * @brief A class to support a pageable display on 1306 OLED displays.
 */
class PagingOLEDDisplay {
    public:

        /**
         * @brief Construct a PagingOLEDDisplay
         * @param width width of the display in pixels
         * @param heigh height of the display in pixels
         * @param pagingPin pin number to use to control pagingPin
         */
        PagingOLEDDisplay(const int width, const int height, const unsigned int pagingPin);

        /**
         * @brief call before use to initialize the device
         * @return true if successfull
         */
        boolean begin();
        
        /** 
         * @brief display the specified page
         * @param page to display
         */
        void displayPage(unsigned int page);

        /** @brief display the current page */
        void displayCurrentPage();

        /** 
         * @brief set the line to display 
         * @param line the line number 
        */
        void setLine(unsigned int line, char* text);

        char displayBuffer_[4][64];     /** Warning: this is bad, we're hard coding for now */
        
    protected:

        /** @brief the ISR when the paging pin is pressed */
        friend void pageISR();

    private:
        Adafruit_SSD1306 display_;
        unsigned int currentPage_;
        unsigned int pagingPin_;
        unsigned int maxDisplayPages_;
};

#endif