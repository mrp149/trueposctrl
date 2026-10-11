#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>		// For read() and close()
#include <arpa/inet.h>		// For socket structures and functions
#include <sys/socket.h>
#include "rxtcp.h"

#define SERVER_IP "192.168.2.60"
#define PORT 8001
#define BUFFER_SIZE 1024

static int sock_fd;
static struct sockaddr_in server_addr;

static char buffer[BUFFER_SIZE];
static char clean_buffer[BUFFER_SIZE];

static ssize_t bytes_read;
static ssize_t clean_idx = 0, read_idx = 0;
static int debug = 0;

int
rxtcp_init ()
{

  // Create the TCP socket
  sock_fd = socket (AF_INET, SOCK_STREAM, 0);
  if (sock_fd < 0)
    {
      perror ("Socket creation failed");
      exit (EXIT_FAILURE);
    }

  // Configure the server address structure
  memset (&server_addr, 0, sizeof (server_addr));
  server_addr.sin_family = AF_INET;
  server_addr.sin_port = htons (PORT);	// Convert port to network byte order

  // Convert IP address from text to binary format
  if (inet_pton (AF_INET, SERVER_IP, &server_addr.sin_addr) <= 0)
    {
      perror ("Invalid address or address not supported");
      close (sock_fd);
      exit (EXIT_FAILURE);
    }

  // Connect to the remote TCP server
  if (connect
      (sock_fd, (struct sockaddr *) &server_addr, sizeof (server_addr)) < 0)
    {
      perror ("Connection to server failed");
      close (sock_fd);
      exit (EXIT_FAILURE);
    }
  printf ("Successfully connected to the server.\n");
  return 0;
}

// Read from TCP port
int
rxtcp_rx ()
{

  while (read_idx >= clean_idx)
    {
      memset (buffer, 0, BUFFER_SIZE);
      clean_idx = 0;

      bytes_read = read (sock_fd, buffer, BUFFER_SIZE - 1);

      if (bytes_read < 0)
	{
	  perror ("Error while reading from socket");
	  close (sock_fd);
	  exit (EXIT_FAILURE);
	}
      else if (bytes_read == 0)
	{
	  printf ("Server closed the connection.\n");
	  close (sock_fd);
	  exit (EXIT_FAILURE);
	}
      else
	{
	  for (ssize_t i = 0; i < bytes_read; i++)
	    {
	      // Check if we hit a Telnet command indicator (0xFF)
	      if ((unsigned char) buffer[i] == 0xFF)
		{
		  // Skip this byte and the next 2 command/option bytes
		  i += 2;
		  continue;
		}
	      clean_buffer[clean_idx++] = buffer[i];
	    }
	  clean_buffer[clean_idx] = '\0';
	  read_idx = 0;
	  // Only print if there is text left over after filtering out the Telnet noise
	  if (debug)
	    {
	      if (clean_idx > 0)
		{
		  printf ("Received %ld bytes: %s", clean_idx, clean_buffer);
		  for (int j = 0; j < clean_idx; j++)
		    printf ("%02x ", (unsigned char) clean_buffer[j]);
		  printf ("\n");
		}
	    }
	}
    }				// while
  return clean_buffer[read_idx++];
}

// Write to TCP port
int
rxtcp_tx (const char *msg, size_t ln)
{

  if (sock_fd > 0)
    return write (sock_fd, msg, ln);
  return 0;
}

int
rxtcp_stop ()
{
  // Close the socket connection
  close (sock_fd);
  sock_fd = -1;
  return 0;
}

#ifdef TEST_RXTCP

int
main ()
{
  char msg[] = { '$', 'G', 'E', 'T', 'P', 'O', 'S', '\n' };
  int ch;

  rxtcp_init ();

  while ((ch = rxtcp_rx ()) != 0xa)
    {
      printf ("%c", ch);
    }
  printf ("%c", ch);

  while ((ch = rxtcp_rx ()) != 0xa)
    {
      printf ("%c", ch);
    }
  printf ("%c", ch);

  printf ("Writing: %s", msg);
  rxtcp_tx (msg, sizeof (msg));

  for (int k = 0; k < 20; k++)
    {
      while ((ch = rxtcp_rx ()) != 0xa)
	{
	  printf ("%c", ch);
	}
      printf ("%c", ch);
    }

  rxtcp_stop ();
  return 0;
}
#endif
