#include <control/control.h>
#include <heat/heat.h>
#include <ssi/ssi.h>
#include <thermal/thermal.h>
#include <tools/break_on.h>
#include <tools/status.h>

#include <stdlib.h>

int main(void) {
  do {
    BREAK_ON_ERROR(HmInitialize());
    BREAK_ON_ERROR(TmInitialize());
    BREAK_ON_ERROR(SsiInitialize());
    BREAK_ON_ERROR(CtrlInitialize());

    for (;;) {
      BREAK_ON_ERROR(CtrlProcessRequests());
    }

  } while (false);
  return 0;
}
