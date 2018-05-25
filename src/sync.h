#ifndef SYNC_H
#define SYNC_H

typedef enum {SYNC_STATE_POSITIVE, SYNC_STATE_NEGATIVE} sync_state;

typedef struct
{
  sync_state state;
}st_sync;

void sync_init(void);
void sync_update_state(void);
void sync_state_changed(void);

#endif
