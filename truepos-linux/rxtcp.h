#ifndef RXTCP_H_
#define RXTCP_H_

int rxtcp_init();
int rxtcp_stop();
int rxtcp_tx(const char *msg, size_t ln);
int rxtcp_rx();

#endif
