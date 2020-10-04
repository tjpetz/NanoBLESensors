#ifndef PAGINGOLEDDISPLAY_H
#define PAGINGOLEDDISPLAY_H

#define NO_ADAFRUIT_SSD1306_COLOR_COMPATIBILIT
#include <Adafruit_SSD1306.h>

// need to undefine these Arduino macros to use std::vector.
#undef min
#undef max
#include <string>
#include <vector>

/**
 * @brief A class to support a pageable display on 1306 OLED displays.
 * @note At this point the class is limited to a fixed number of lines
 *       and works out the lines per pages on fixed font sizes.
 */
class PagingOLEDDisplay {
public:
  /**
   * @brief Construct a PagingOLEDDisplay
   * @param width width of the display in pixels
   * @param height height of the display in pixels
   * @param nbrOfLines is the total number of lines, the display will split this
   * into pages
   * @param pagingPin pin number to use to control pagingPin
   */
  PagingOLEDDisplay(unsigned int width, unsigned int height,
                    unsigned int nbrOfLines, unsigned int pagingPin);

  /**
   * @brief call before use to initialize the device
   * @return true if successfull
   */
  boolean begin();

  /**
   * @brief empty the line buffer and clear the display
   */
  void clear();

  /**
   * @brief print to a specified line using printf formatting.
   * @param line the line to print
   * @param fmt the printf format string
   * @param ... the printf parameters
   */
  void printf(unsigned int line, const char *fmt, ...);

  /**
   * @brief display the specified page
   * @param page to display
   */
  void displayPage(unsigned int page);

  /** @brief display the current page */
  void displayCurrentPage();

protected:
  /** @brief the ISR when the paging pin is pressed */
  friend void pageISR();

private:
  Adafruit_SSD1306 display_;
  unsigned int displayHeight_;
  unsigned int displayWidth_;
  unsigned int nbrOfLines_;
  unsigned int linesPerPage_;
  unsigned int currentPage_;
  unsigned int pagingPin_;
  unsigned int nbrOfPages_;
  unsigned int lineHeight_;
  std::vector<std::string> lineBuffer_;
};

#endif
