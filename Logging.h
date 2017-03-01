#ifndef __LOGGING__
#define __LOGGING__

struct Log {
  uint32_t process_time;

  struct {
    int16_t sv_major;   // 目標値
    int16_t pv_major;   // 測定値
    int16_t sv_minor;
    int16_t pv_minor;
    int16_t mv;   // 操作量
    int16_t sat;  // 飽和操作量
  } m[2];

  size_t printTo(Print& p) const {
    static String msg;
    msg = "";

    msg += m[0].sv_major; msg += '\t';
    msg += m[0].pv_major; msg += '\t';    
    msg += m[0].sv_minor; msg += '\t';
    msg += m[0].pv_minor; msg += '\t';
    msg += m[0].mv; msg += '\t';
    msg += m[0].sat; msg += '\t';

    msg += m[1].sv_major; msg += '\t';
    msg += m[1].pv_major; msg += '\t';
    msg += m[1].sv_minor; msg += '\t';
    msg += m[1].pv_minor; msg += '\t';
    msg += m[1].mv; msg += '\t';
    msg += m[1].sat; msg += '\t';

    p.println(msg);
  }
};

#endif

