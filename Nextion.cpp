/*! \file */

#include "Nextion.h"
#include "INextionTouchable.h"


void NextionMsgLst::enqMsg(Buffer &buffer)
{
  if (buffer[0] == NEX_RET_EVENT_TOUCH_HEAD)
  {
    //printf("Push Touch Event %d %d %d\n", int(buffer[1]), int(buffer[2]), int(buffer[3]));
    uint8_t nwr = ewr+1;
    if (nwr >= NextionMsgLstEvts)
      nwr = 0;
    if ((nwr == erd) && elw)	// Full ??
    {
      printf("Touch Event Queue FULL!!\n");
      return;
    }
    Evt[ewr].page = buffer[1];
    Evt[ewr].cmpnt = buffer[2];
    Evt[ewr].event = NextionEventType(buffer[3]);
    ewr=nwr;
    elw = true;
  }
  else
  {
    if (NextionValue(buffer[0]) <= NEX_RET_VAR_NAME_TOO_LONG)
    {
      //printf("Don't push message 0x%02x!\n", int(buffer[0])); //as cmd [n]ack are not meanfull out of context!
      return;
    }
    printf("Push Msg 0x%02x\n", int(buffer[0]));
    uint8_t nwr = mwr+1;
    if (nwr >= NextionMsgLstMsgs)
      nwr = 0;
    if ((nwr == mrd) && mlw)	// Full ??
    {
      printf("Msg Queue FULL!!\n");
      return;
    }
    Msg[mwr].msg = NextionValue(buffer[0]);
    Msg[mwr].str = buffer.substring(1, buffer.length() - 3);
    Msg[mwr].datalen = Msg[mwr].str.length();
    mwr=nwr;
    mlw = true;
  }
}

bool NextionMsgLst::haveEvt(void)
{
  return (erd != ewr) || elw;
}

bool NextionMsgLst::haveMsg(void)
{
  return (mrd != mwr) || mlw;
}

NextionMsg *NextionMsgLst::deqMsg(void)
{
  if (!haveMsg())
    return NULL;
  NextionMsg *msg = &Msg[mrd++];
  if (mrd >= NextionMsgLstMsgs)
    mrd = 0;
  mlw = false;
  return msg;
}

NextionEvt *NextionMsgLst::deqEvt(void)
{
  if (!haveEvt())
    return NULL;
  NextionEvt *evt = &Evt[erd++];
  if (erd >= NextionMsgLstEvts)
    erd = 0;
  elw = false;
  return evt;
}

/*!
 * \brief Creates a new device driver.
 * \param stream Stream (serial port) the device is connected to
 * \param flushSerialBeforeTx If the serial port should be flushed before
 *                            transmission
 */
Nextion::Nextion(Stream &stream, bool flushSerialBeforeTx)
    : m_serialPort(stream)
    , m_timeout(500)
    , m_flushSerialBeforeTx(flushSerialBeforeTx)
    , m_touchableList(NULL)
    , m_function2(NULL)
{
  m_serialPort.setTimeout(m_timeout);
}

/*!
 * \brief Initialises the device.
 * \return True if initialisation was successful.
 */
bool Nextion::init()
{
  sendCommand("");	// Generate a EOF marker to reset device cmd parser

  sendCommand("bkcmd=3");
  bool result1 = checkCommandComplete();

  sendCommand("page 0");
  bool result2 = checkCommandComplete();

  MsgLst.Purge();

  return (result1 && result2);
}

/*!
 * \brief Polls for new messages and touch events.
 */
void Nextion::poll()
{
  while (MsgLst.haveEvt() || MsgLst.haveMsg() || enqueuMsg())
  {
    NextionMsg *Msg = MsgLst.deqMsg();
    if (Msg)
    {
      ; //printf("Drop Msg 0x%02x\n", int(Msg->msg));
    }

    NextionEvt *Evt = MsgLst.deqEvt();
    if (Evt)
    {
      printf("Dispatch event 0x%02x on %d.%d\n", int(Evt->event), int(Evt->page), int(Evt->cmpnt));
      ITouchableListItem *item = m_touchableList;
      while (item != NULL)
      {
	if (item->item->processEvent(Evt->page, Evt->cmpnt, Evt->event))
	  break;
	item = item->next;
      }
      if ((!item) && (m_function2))
      {
	m_function2(Evt->event, Evt->page, Evt->cmpnt);
      }
    }
  }
}
/*!
 * \brief Attaches a callback function to this widget.
 * \param function Pointer to callback function
 * \return True if successful
 * \see INextionTouchable::detachCallback
 */
bool Nextion::attachFunction(NextionFunction2 fct)
{
  if (!fct)
    return false;

  if (m_function2 != NULL)
    detachFunction();

  m_function2 = fct;
  return true;
}

/*!
 * \brief Removes the callback handler from this widget
 *
 * Memory is not freed.
 */
void Nextion::detachFunction()
{
  m_function2 = NULL;
}

/*!
 * \brief Refreshes the entire page.
 * \return True if successful
 */
bool Nextion::refresh()
{
  sendCommand("ref 0");
  return checkCommandComplete();
}

/*!
 * \brief Refreshes a specific object.
 * \param objectName Name of the object to refresh
 * \return True if successful
 */
bool Nextion::refresh(const String &objectName)
{
  sendCommand("ref %s", objectName.c_str());
  return checkCommandComplete();
}

/*!
 * \brief Puts the device into sleep mode.
 * \return True if successful
 */
bool Nextion::sleep()
{
  sendCommand("sleep=1");
  return checkCommandComplete();
}

/*!
 * \brief Wakes the device from sleep mode.
 * \return True if successful
 */
bool Nextion::wake()
{
  sendCommand("sleep=0");
  return checkCommandComplete();
}

bool Nextion::setGlobal(const String &varName, uint32_t number)
{
  sendCommand("%s=%lu", varName.c_str(), number);
  return checkCommandComplete();
}

bool Nextion::getGlobal(const String &varName, uint32_t *number)
{
  sendCommand("get %s", varName.c_str());
  return receiveNumber(number);
}
/*!
 * \brief Gets the current backlight brightness.
 * \return Brightness
 */
uint16_t Nextion::getBrightness()
{
  sendCommand("get dim");
  uint32_t val;
  if (receiveNumber(&val))
    return val;
  else
    return 0;
}

/*!
 * \brief Sets the backlight brightness.
 * \param val Brightness value (0-100)
 * \param persist If set to true value will be set as new power on default
 * \return True if successful
 */
bool Nextion::setBrightness(uint16_t val, bool persist)
{
  if (persist)
  {
    sendCommand("dims=%d", val);
  }
  else
  {
    sendCommand("dim=%d", val);
  }
  return checkCommandComplete();
}

/*!
 * \brief Gets the ID of the current displayed page.
 * \return Page ID
 */
uint8_t Nextion::getCurrentPage()
{
  sendCommand("sendme");

  Buffer msg;
  if (searchMsg(NEX_RET_CURRENT_PAGE_ID_HEAD, msg))
  {
    return (const uint8_t)msg[1];
  }
  printf("Can't get current page !!!\n");

  return 0;
}

/*!
 * \brief Clears the current display.
 * \param colour Colour to set display to
 * \return True if successful
 */
bool Nextion::clear(uint32_t colour)
{
  sendCommand("cls %d", colour);
  return checkCommandComplete();
}

/*!
 * \brief Draws a pre uploaded picture on the display.
 * \param x X position
 * \param y Y position
 * \param id ID of the picture to display
 * \return True if successful
 */
bool Nextion::drawPicture(uint16_t x, uint16_t y, uint8_t id)
{
  sendCommand("pic %d,%d,%d", x, y, id);
  return checkCommandComplete();
}

/*!
 * \brief Draws a cropped pre uplaoded picture on the display.
 * \param x X position
 * \param y Y position
 * \param w Width
 * \param h Height
 * \param id ID of the picture to display
 * \return True if successful
 */
bool Nextion::drawPicture(uint16_t x, uint16_t y, uint16_t w, uint16_t h,
                          uint8_t id)
{
  sendCommand("picq %d,%d,%d,%d,%d", x, y, w, h, id);
  return checkCommandComplete();
}

/*!
 * \brief Draws a string on the display.
 * \param x X position
 * \param y Y position
 * \param w Width
 * \param h Height
 * \param fontID ID of the font to use
 * \param str String to draw
 * \param bgColour Colour of the background of the bounding box
 * \param fgColour Colour of the text
 * \param bgType Background type
 * \param xCentre X alignment
 * \param yCentre Y alignment
 * \return True if successful
 */
bool Nextion::drawStr(uint16_t x, uint16_t y, uint16_t w, uint16_t h,
                      uint8_t fontID, const String &str, uint32_t bgColour,
                      uint32_t fgColour, uint8_t bgType,
                      NextionFontAlignment xCentre,
                      NextionFontAlignment yCentre)
{
  sendCommand("xstr %d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%s", x, y, w, h, fontID, fgColour, bgColour, xCentre, yCentre, bgType, str.c_str());
  return checkCommandComplete();
}

/*!
 * \brief Draws a line on the display.
 * \param x1 X position of first vertex
 * \param y1 Y position of first vertex
 * \param x2 X position of second vertex
 * \param y2 Y position of second vertex
 * \param colour Colour
 * \return True if successful
 */
bool Nextion::drawLine(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2,
                       uint32_t colour)
{
  sendCommand("line %d,%d,%d,%d,%d", x1, y1, x2, y2, colour);
  return checkCommandComplete();
}

/*!
 * \brief Draws a rectangle on the display.
 * \param x X position
 * \param y Y position
 * \param w Width
 * \param h Height
 * \param filled If the rectangle should be filled with a solid colour
 * \param colour Colour
 * \return True if successful
 */
bool Nextion::drawRect(uint16_t x, uint16_t y, uint16_t w, uint16_t h,
                       bool filled, uint32_t colour)
{
  if (filled)
  {
    sendCommand("draw %d,%d,%d,%d,%d", x, y, x + w, y + h, colour);
  }
  else
  {
    sendCommand("fill %d,%d,%d,%d,%d", x, y, x + w, y + h, colour);
  }
  return checkCommandComplete();
}

/*!
 * \brief Draws a circle on the display.
 * \param x X position
 * \param y Y position
 * \param r Radius
 * \param colour Colour
 * \return True if successful
 */
bool Nextion::drawCircle(uint16_t x, uint16_t y, uint16_t r, uint32_t colour)
{
  sendCommand("cir %d,%d,%d,%d", x, y, r, colour);
  return checkCommandComplete();
}

/*!
 * \brief Adds a INextionTouchable to the list of registered touchable
 *        elements.
 * \param touchable Pointer to the INextionTouchable
 *
 * Required for touch events from an INextionTouchable to be polled.
 *
 * Should be called automatically by INextionTouchable::INextionTouchable.
 */
void Nextion::registerTouchable(INextionTouchable *touchable)
{
  ITouchableListItem *newListItem = new ITouchableListItem;
  newListItem->item = touchable;
  newListItem->next = NULL;

  if (m_touchableList == NULL)
  {
    m_touchableList = newListItem;
  }
  else
  {
    ITouchableListItem *item = m_touchableList;
    while (item->next != NULL)
      item = item->next;
    item->next = newListItem;
  }
}

/*!
 * \brief Sends a command to the device.
 * \param command Command to send
 */
void Nextion::sendCommand(const String &command)
{
  /*
  if (m_flushSerialBeforeTx)
  {
    while(m_serialPort.available())
    {
      m_serialPort.read();
    }
  }
  */

  m_serialPort.print(command);
  m_serialPort.write(0xFF);
  m_serialPort.write(0xFF);
  m_serialPort.write(0xFF);

  //printf("Command '%s' sent\n", command.c_str());
}

void Nextion::sendCommand(const char *format, ...)
{
  va_list args;
  va_start(args, format);
  sendCommand(format, args);
  va_end(args);
}

void Nextion::sendCommand(const char *format, va_list args)
{
  char buf[512] = {0};
  vsnprintf(buf, sizeof(buf), format, args);
  sendCommand(String(buf));
}

/*!
 * \brief Checks if the last command was successful.
 * \return True if command was successful
 */
bool Nextion::checkCommandComplete()
{
  uint32_t deadline = millis() + m_timeout;
  Buffer msg;
  do
  {
    if (receiveMsg(msg, deadline))
    {
      NextionValue hdr = NextionValue(msg[0]);
      if (hdr == NEX_RET_CMD_FINISHED) 
      {
	//printf("Command OK\n");
	return true;
      }
      else if (hdr <= NEX_RET_VAR_NAME_TOO_LONG)
      {
	printf("Nextion::checkCommandComplete, error 0x%02X\n", int(hdr));
	return false;
      }
      else
	MsgLst.enqMsg(msg);
    }
  } while (millis() < deadline);
  printf("Command without answer\n");

  return false;
}

/*!
 * \brief Receive a number from the device.
 * \param number Pointer to the number to store received number in
 * \return True if receive was successful
 */
bool Nextion::receiveNumber(uint32_t *number)
{
  if (!number)
    return false;

  Buffer msg;
  if (searchMsg(NEX_RET_NUMBER_HEAD, msg))
  {
    const uint8_t *temp = (const uint8_t*)msg.begin();
    *number = (uint32_t(temp[4]) << 24) | (uint32_t(temp[3]) << 16) | (uint16_t(temp[2]) << 8) | (temp[1]);
    return true;
  }
  printf("Can't get INTEGER\n");

  return false;
}

/*!
 * \brief Receive a string from the device.
 * \param buffer Pointer to buffer to store string in
 * \return Actual length of string received
 */
size_t Nextion::receiveString(String &buffer, bool stringHeader)
{
  Buffer msg;
  if (searchMsg(NEX_RET_NUMBER_HEAD, msg))
  {
    buffer = msg.substring(1,msg.length()-3).c_str();
    return buffer.length();
  }
  printf("Can't get STRING\n");
  return 0;
}

/*
void Nextion::enqueu(void)
{
  uint32_t deadline =  millis() + m_timeout;
  Buffer msg;
  while (receiveMsg(msg, deadline))
  {
    MsgLst.enqMsg(msg);
  }
}
*/

bool Nextion::enqueuMsg(void)
{
  //printf("Nextion::enqueuMsg...\n");
  Buffer msg;
  uint32_t deadline = millis()+5;
  if (receiveMsg(msg, deadline))
  {
    MsgLst.enqMsg(msg);
    return true;
  }
  return false;
}


uint8_t NextionParser::getMsgLen(uint8_t hdr)
{
  switch (hdr)
  {
    case NEX_RET_INVALID_CMD: return(4+2);	// mays be 1+3 (invalid command) or 3+3 (Startup) , take longer one
    case NEX_RET_EVENT_TOUCH_HEAD: return(4+3);
    case NEX_RET_CURRENT_PAGE_ID_HEAD: return(4+1);
    case NEX_RET_EVENT_POSITION_HEAD:
    case NEX_RET_EVENT_SLEEP_POSITION_HEAD: return(4+5);
    case NEX_RET_STRING_HEAD: return(0);	// Variable
    case NEX_RET_NUMBER_HEAD: return(4+4);	// May generate false EOF
    default: return(4+0);
  }
}

bool Nextion::searchMsg(NextionValue value, Buffer &msg, uint32_t deadline)
{
  if (!deadline)
    deadline = millis() + m_timeout;
  do
  {
    if (receiveMsg(msg, deadline))
    {
      if (NextionValue(msg[0]) == value)
      {
	//printf("Found message type 0x%02x\n", value);
	return true;
      }
      MsgLst.enqMsg(msg);
    }
  } while (millis() < deadline);
  printf("Nextion::searchMsg(0x%02x): can't find message !!\r\n", int(value));
  return false;
}

void dumpit(const Buffer &buf)
{
  printf("Msg(%d): ", buf.length());
  for (uint8_t i = 0; i < buf.length(); ++i)
    printf("%02X ", buf[i] & 0xFF);
  printf("\n\r");
}


// 0xFF 0xFF 0xFF is the EOF (end of frame) marker. This marker can not be present in the message content except
// for Numeric Data message (0x71) where we can get up to four consecutive 0xff before the EOF marker.
bool Nextion::receiveMsg(Buffer &msg, uint32_t deadline)
{
  if (!deadline)
    deadline =  millis() + m_timeout;

  return m_Parser.receiveMsg(m_serialPort, msg, deadline);
}

bool NextionParser::receiveMsg(Stream &serialPort, Buffer &msg, uint32_t deadline)
{
  // Handle pending message
  do {
    if (parsePos(msg, currPos))	// In the loop because parse may "generate" message in reparsing bad one ???
      return true;

    while (serialPort.available())
    {
      m_buffer.concat(char(serialPort.read()));
      if (parsePos(msg, currPos))
	return true;
    }
  } while (millis() <= deadline);

  if (m_buffer.length())
  {
    //printf("NextionParse::receiveMsg(): remind  bytes in buffer : "); dumpit(m_buffer);
  }

  return false;
}

// 0xFF 0xFF 0xFF is the EOF (end of frame) marker. This marker can not be present in the message content except
// for Numeric Data message (0x71) where we can get up to four consecutive 0xff before the EOF marker.
bool NextionParser::parsePos(Buffer &msg, uint8_t startPos)
{
  uint8_t pos = startPos;
  while (pos < m_buffer.length())
  {
    uint8_t c = m_buffer[pos];
    if (!pos)
    {
      if (c == 0xFF)	// 0xFF can't be a header
      {
	printf("\nNextionParser::parse(): should not have message type 0xFF!!\n");
	m_buffer.remove(0,1);
	continue;
      }
      hdr = c;
      maxlen = getMsgLen(hdr);
      if (maxlen)
	m_buffer.reserve(maxlen);
      else
	m_buffer.reserve(64);	// base for strings
    }
    else
    {
      if (c == 0xFF)
	flag_count++;
      else 
	flag_count = 0;
    }
    pos++;

    while (flag_count == 3) // Got a EOF
    {
      uint8_t blen  = pos;	// block len
      if ((hdr == NEX_RET_NUMBER_HEAD) && (blen < maxlen)) // Handle exception of NUMERIC value where EOF may not be where we want
      {
	flag_count--; // Remove a false 0xFF marker
	break;
      }

      if ((hdr == NEX_RET_INVALID_CMD) && (blen >= (1+3))) 	// Alternative length for thr Msg header
	maxlen=(1+3);	// now it's my len

      if (!maxlen) // no limit, i.e. STRING
	maxlen=blen; // length is current one

      if (blen == maxlen) // theorical length == pratical length
      {
	if (maxlen == m_buffer.length())
	  msg = m_buffer;
	else
	  msg = m_buffer.substring(0, maxlen);
	m_buffer.remove(0,maxlen);
	currPos = 0;
	flag_count = 0;
	//printf("Got message : "); dumpit(msg);
	return true;
      }

      // shoud not appen in perfect world: now we have a problem. We must to find a solution.
      // Then first char is not a header and drop it
      if (blen <= 4)	// a message must have four char minimum including EOF marker then drop this garbage
      {
	printf("\nNextionParser::parse(): too short: drop corrupted message!! "); dumpit(m_buffer);
	m_buffer.remove(0,blen);
	pos = 0;
	flag_count = 0;
      }
      else	// reevaluate with new header and shorter message
      {
	printf("\nNextionParser::parser(): bad size: drop header of malformed message!! "); dumpit(m_buffer);
	m_buffer.remove(0,1);	// remove false header
	pos--;			// we are shorter
	hdr = m_buffer[0];	// new header
	maxlen=getMsgLen(hdr);	// and new expected message size
      }
    } // while EOF
    if ((maxlen != 0) && (pos >= maxlen))	// We should have reach EOF here !!
    {
      printf("\nNextionParser::parser(): tool long : drop header of malformed message!!"); dumpit(m_buffer);
      m_buffer.remove(0,1);	// remove false header
      pos--;			// we are shorter
      hdr = m_buffer[0];	// new header
      maxlen=getMsgLen(hdr);	// and new expected message size
    }
  }
  currPos = pos;

  return false;
}
