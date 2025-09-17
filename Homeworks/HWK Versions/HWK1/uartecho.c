/*
 *  MSP432E401Y serial command shell
 *  Echoes every received character back out
 *  Recognizes:
 *      -help  : prints command list
 *      -about : prints programmer / assignment info
 *
 *  Keep the ABOUT_* strings and APP_VERSION updated.
 *
 *  Build: CCS 12.8.0 + SimpleLink MSP432E4 SDK 4.x
 */
#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>
#include <string.h>
#include <ctype.h>

#include <ti/drivers/GPIO.h>
#include <ti/drivers/UART.h>
#include "ti_drivers_config.h"

/* --------Data to maintain -------- */
#define ABOUT_NAME       "Salim Sadman Bishal"
#define ABOUT_ASSIGNMENT "ECE 5380 HWK1"
#define APP_VERSION      "v1.2.0"
/* -------------------------------------------- */

#define BUILD_DATE       __DATE__
#define BUILD_TIME       __TIME__

/* shell settings */
#define RX_BUF_SZ 64            /* ring-buffer size */
#define MAX_CMD_LEN 32      /* longest legal command line     */

/* ------------ file-scope (global-static variables ------------ */
static UART_Handle gUart;
static char gLineBuf[RX_BUF_SZ];
static bool overflow = false;  /* set when user types too long   */

/* simple blocking transmit */
static void putStr(const char *s)
{
    UART_write(gUart, s, strlen(s));
}

static void banner(void)
{
    putStr("\r\n*** MSP432 Command Shell Ready ***\r\n"
           "Type -help for a list of available commands.\r\n\r\n");
}

/* command handlers */
static void cmd_about(void)
{
    char msg[128];
    snprintf(msg, sizeof(msg),
             "%s | %s | %s | built %s %s\r\n",
             ABOUT_NAME, ABOUT_ASSIGNMENT, APP_VERSION,
             BUILD_DATE, BUILD_TIME);
    putStr(msg);
}

static void cmd_help(void)
{
    putStr("Supported commands:\r\n"
           "  -about   Show version / author info\r\n"
           "  -help    help\r\n");
}

static void handleLine(char *line)
{
    /* echo full line */
    putStr(line);
    putStr("\r\n");

    if (line[0] != '-') return;

    if (!strcmp(line, "-about"))
        cmd_about();
    else if (!strcmp(line, "-help"))
        cmd_help();
    else
        putStr("?? unknown command\r\n");
}

/* ======== mainThread ======== */
void *mainThread(void *arg0)
{
    char ch;
    size_t idx = 0;

    GPIO_init();
    UART_init();

    GPIO_write(CONFIG_GPIO_LED_0, CONFIG_GPIO_LED_ON);

    UART_Params params;
    UART_Params_init(&params);
    params.baudRate       = 115200;
    params.readDataMode   = UART_DATA_BINARY;
    params.writeDataMode  = UART_DATA_BINARY;
    params.readReturnMode = UART_RETURN_FULL;

    gUart = UART_open(CONFIG_UART_0, &params);
    if (!gUart) while (1);

    banner();

    for (;;)
    {
        UART_read(gUart, &ch, 1);      /* wait one byte */
        UART_write(gUart, &ch, 1);     /* immediate echo */

        if (ch == '\r' || ch == '\n')
        {
            if(overflow){
                putStr("\r\n!! error: line too long (max 31 chars)\r\n");
                overflow = false;
                idx = 0; /* flush the buffer      */
                continue;
            }

            if (idx > 0)
            {
                gLineBuf[idx] = '\0';
                handleLine(gLineBuf);
                idx = 0;
            }
        }
        else
        {
            if ((ch == 0x08 || ch == 0x7F))          /* 0x08 = BS, 0x7F = DEL */
            {
                if (idx > 0) {
                    idx--;
                    /* erase char on PuTTY: back, space, back */
                    const char bsSeq[] = "\b \b";
                    UART_write(gUart, bsSeq, sizeof(bsSeq) - 1);
                }
                continue;
            }

            if (overflow) {
                continue;
            }


            if (idx < MAX_CMD_LEN - 1) {
                gLineBuf[idx++] = ch;
            }
            else {
                putStr("\r\n!! character-overflow\r\n");
                overflow = true;
            }
        }
    }
}

