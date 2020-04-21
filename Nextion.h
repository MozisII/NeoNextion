#ifndef __NEONEXTION_NEXTION
#define __NEONEXTION_NEXTION

#if defined(SPARK) || defined(PLATFORM_ID)
#include "application.h"
extern char *itoa(int a, char *buffer, unsigned char radix);
#else
#include <Arduino.h>
#endif

#include <WString.h>
#include <WBuffer.h>

#include "NextionTypes.h"

#include <LibPrintf.h>


class INextionTouchable;
class Nextion;

/*!
 * \struct ITouchableListItem
 * \brief Linked list node for INextionTouchable objects.
 */
struct ITouchableListItem
{
  INextionTouchable *item;  //!< Pointer to stored INextionTouchable
  ITouchableListItem *next; //!< Pointer to next list node
};


/*
 *
 *
 */

struct NextionEvt{
  uint8_t page;
  uint8_t cmpnt;
  NextionEventType event;
};
struct NextionMsg
{
  NextionValue msg;
  uint8_t datalen;
  Buffer str;

  NextionMsg() : msg(NEX_RET_INVALID_CMD), datalen(0) {}
  ~NextionMsg() { datalen=0;}
  //uint8_t& operator [](uint8_t idx) { if (idx >= datalen) return buf[datalen-1]; return buf[idx]; }
};

struct NextionParser
{
  bool have_header_flag;
  uint8_t hdr;
  uint8_t maxlen;
  uint8_t flag_count;
  uint8_t currPos;
  Buffer m_buffer;

  bool receiveMsg(Stream &serialPort, Buffer &msg, uint32_t deadline);
  bool parsePos(Buffer &msg, uint8_t startPos);
  //bool parseChar(Buffer &msg, uint8_t c);
  void clear(void) { have_header_flag=0; hdr = currPos = maxlen = flag_count = 0; m_buffer.clear(); };
  uint8_t getMsgLen(uint8_t hdr);
};



const int NextionMsgLstEvts = 8;
const int NextionMsgLstMsgs = 8;
struct NextionMsgLst
{
  NextionEvt Evt[NextionMsgLstEvts];
  NextionMsg Msg[NextionMsgLstMsgs];
  uint8_t erd, ewr;
  uint8_t mrd, mwr;
  bool elw, mlw;
  NextionMsgLst() : erd(0), ewr(0), mrd(0), mwr(0), elw(false), mlw(false) { }
  //~NextionMsgLst() { }
  void purgeEvt(void) { erd=ewr=0; elw=false;}
  void purgeMsg(void) { mrd=mwr=0; mlw=false;}
  void Purge(void) {purgeEvt(); purgeMsg();}
  void enqMsg(Buffer &);
  NextionMsg *deqMsg(void);
  NextionEvt *deqEvt(void);
  bool haveEvt(void);
  bool haveMsg(void);
};



/*!
 * \class Nextion
 * \brief Driver for a physical Nextion device.
 */
class Nextion
{
  friend NextionMsgLst;
public:
  typedef void (*NextionFunction2)(NextionEventType, uint8_t page, uint8_t component);
  
  Nextion(Stream &stream, bool flushSerialBeforeTx = true);

  bool init();
  void poll();

  bool refresh();
  bool refresh(const String &objectName);

  bool sleep();
  bool wake();

  bool setGlobal(const String &varName, uint32_t number);
  bool getGlobal(const String &varName, uint32_t *number);

  uint16_t getBrightness();
  bool setBrightness(uint16_t val, bool persist = false);

  uint8_t getCurrentPage();

  bool clear(uint32_t colour = NEX_COL_WHITE);
  bool drawPicture(uint16_t x, uint16_t y, uint8_t id);
  bool drawPicture(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint8_t id);
  bool drawStr(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint8_t fontID,
               const String &str, uint32_t bgColour = NEX_COL_BLACK,
               uint32_t fgColour = NEX_COL_WHITE,
               uint8_t bgType = NEX_BG_SOLIDCOLOUR,
               NextionFontAlignment xCentre = NEX_FA_CENTRE,
               NextionFontAlignment yCentre = NEX_FA_CENTRE);
  bool drawLine(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2,
                uint32_t colour);
  bool drawRect(uint16_t x, uint16_t y, uint16_t w, uint16_t h, bool filled,
                uint32_t colour);
  bool drawCircle(uint16_t x, uint16_t y, uint16_t r, uint32_t colour);

  void registerTouchable(INextionTouchable *touchable);
  void sendCommand(const String &command);
  void sendCommand(const char *format, ...);
  void sendCommand(const char *format, va_list args);
  bool checkCommandComplete();
  bool receiveNumber(uint32_t *number);
  size_t receiveString(String &buffer, bool stringHeader=true);
  
  bool attachFunction(NextionFunction2 fct);
  void detachFunction();

  NextionMsgLst	MsgLst;

  bool receiveMsg(Buffer &msg, uint32_t deadline = 0);
  bool searchMsg(NextionValue value, Buffer &msg, uint32_t deadline = 0);
  //void enqueu(void);
private:
  Stream &m_serialPort;       //!< Serial port device is attached to
  uint32_t m_timeout;         //!< Serial communication timeout in ms
  bool m_flushSerialBeforeTx; //!< Flush serial port before transmission
  ITouchableListItem *m_touchableList; //!< LInked list of INextionTouchable
  NextionFunction2 m_function2; //!< Default callback
  
  NextionParser m_Parser;
  bool enqueuMsg(void);

};

#endif
