/**
 * @file scroll.c
 * @brief This is a hack to get scrolling output on a TTY (no rio).
 *        Or at least some approximation thereof.
 * 
 * Input is captured, and entirely fills up the tty.
 * The program then waits for a few seconds, after which
 *  it dumps out the next batch of text, and repeats.
 *
 * The original intention was to use notes to have the user
 *  press a key to have the screen scroll. As it turns out,
 *  that functionality is handled by rio. Sucks.
 *
 * @author Sean Stangl (sean.stangl@gmail.com)
 * @date 2009.10.09
 * @bug No known bugs.
 */

#include <u.h>
#include <libc.h>

#define TTY_HEIGHT 25
#define TTY_WIDTH 80
#define TAB_WIDTH 8
#define BUF_SIZE 8192 /* Should be > TTY_HEIGHT * TTY_WIDTH. */

#define DEFAULT_PAUSE_SECONDS 5
#define MAX_PAUSE_SECONDS 30

void
main(int argc, char *argv[])
{
	int i, n;
	int nrows;
	int ncols;
	char buf[BUF_SIZE];
	int buf_base;
	int pause_seconds = DEFAULT_PAUSE_SECONDS;

	/* Let's take some arguments, I GUESS.
	 * The only permissible argument is the # of seconds to pause. */
	if (argc > 2)
		sysfatal("Error: Too many arguments.");
	else if (argc == 2)
	{
		pause_seconds = atoi(argv[1]);
		if (pause_seconds < 0 || pause_seconds > MAX_PAUSE_SECONDS)
			sysfatal("Error: Seconds must be in range [0..%d]", MAX_PAUSE_SECONDS);
	}

	while ((n = read(0, buf, (long)sizeof buf)) > 0)
	{
		buf_base = 0;
		nrows = 0;
		ncols = 0;

		for (i = 0; i < n && buf[i]; i++)
		{
			/* Process the next character. */
			if (buf[i] == '\n')
			{
				nrows++;
				ncols = 0;
			}
			else if (buf[i] == '\t')
				ncols = (ncols + TAB_WIDTH) % TAB_WIDTH;
			else
				ncols++;

			/* Did this cause any stopping conditions to hit? */
			if (ncols >= TTY_WIDTH)
			{
				nrows++;
				ncols = ncols % TTY_WIDTH;
			}
			if (nrows >= TTY_HEIGHT - 1)
			{
				/* Stay a while, and listen. */
				write(1, &buf[buf_base], i+1 - buf_base);
				sleep(1000 * pause_seconds);
				
				/* Reset state. Fresh terminal approaching! */
				buf_base = i+1;
				nrows = 0;
			}

		}

		/* Is there anything left in the buffer to dump? */
		if (buf_base < n)
			write(1, &buf[buf_base], n - buf_base);

	}

	if (n < 0)
		sysfatal("Input Error: %r");

	exits(nil);
}

