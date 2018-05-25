#include <stddef.h>
#include "sync.h"

st_sync sync;

void sync_init(void)
{
  sync.state = SYNC_STATE_NEGATIVE;
}

  /*Determines actual polarity of supply voltage and changes sync.state if nessesary*/
void sync_update_state(void)
{

}

  /*Function is called when sync.state changes*/
void sync_state_changed(void)
{

}
