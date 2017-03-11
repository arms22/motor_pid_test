#ifndef __LOGGING__
#define __LOGGING__

struct Log {
  uint32_t process_time;

  struct {
    int16_t sv;   // 目標値
    int16_t pv;   // 測定値
    int16_t mv;   // 操作量
    int16_t sat;  // 飽和操作量

    int16_t work1;
    int16_t work2;
  } m[2];

  size_t printTo(Print& p) const {
    static String msg;
    msg = "";

    msg += m[0].sv; msg += '\t';
    msg += m[0].pv; msg += '\t';
    msg += m[0].mv; msg += '\t';
    msg += m[0].sat; msg += '\t';
    msg += m[0].work1; msg += '\t';
    msg += m[0].work2; msg += '\t';
    
    msg += m[1].sv; msg += '\t';
    msg += m[1].pv; msg += '\t';
    msg += m[1].mv; msg += '\t';
    msg += m[1].sat; msg += '\t';
    msg += m[1].work1; msg += '\t';
    msg += m[1].work2; msg += '\t';

    p.println(msg);
  }
};

#endif

