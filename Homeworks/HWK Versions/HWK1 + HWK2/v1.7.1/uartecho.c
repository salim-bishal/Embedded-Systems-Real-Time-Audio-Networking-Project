/*
 *  MSP432E401Y Enhanced Command-Line Shell (C89-safe) ─ v1.7.1
 *  ────────────────────────────────────────────────────────────────────────────
 *  Key updates vs. v1.7.0
 *    • All `for`loop index variables are now declared at the top of their
 *      enclosing block to satisfy TI/CCS’s default C89 compiler mode.
 *    • No mixed declarations and code; every block begins with declarations.
 *    • Retains the hyphen‑prefixed command syntax and expanded `-memr` help.
 */

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>
#include <string.h>
#include <ctype.h>
#include <stdlib.h>
#include <stdio.h>

#include <ti/drivers/GPIO.h>
#include <ti/drivers/UART.h>
#include "ti_drivers_config.h"

/* ────────────────  METADATA  ─────────────────────────────────────────── */
#define ABOUT_NAME        "Salim Sadman Bishal"
#define ABOUT_ASSIGNMENT  "ECE 5380 HWK1"
#define APP_VERSION       "v1.7.1"
#define BUILD_DATE        __DATE__
#define BUILD_TIME        __TIME__

/* ────────────────  CONFIGURATION  ───────────────────────────────────── */
#define RX_BUF_SZ      64   /* total buffer size (includes headroom) */
#define MAX_CMD_LEN    32   /* 31 printable chars + 1 NUL          */

/* ────────────────  FILE‑SCOPE STATE  ────────────────────────────────── */
static UART_Handle gUart;                /* UART handle returned by UART_open() */
static char       gLineBuf[RX_BUF_SZ];   /* buffer for current input line       */
static size_t     len = 0;               /* number of valid chars in gLineBuf   */
static size_t     cursor = 0;            /* cursor position within [0..len]     */

/* Single‑entry history */
static char history[RX_BUF_SZ] = {0};
static bool hasHistory          = false;

/* ────────────────  UTILITY I/O HELPERS  ─────────────────────────────── */
static void putStr(const char *s)
{
    UART_write(gUart, s, strlen(s));
}

static void putChar(char c)
{
    UART_write(gUart, &c, 1);
}

static void putHex32(uint32_t v)
{
    char buf[11];
    snprintf(buf, sizeof(buf), "0x%08X", v);
    putStr(buf);
}

/* ────────────────  PROMPT & BANNER  ─────────────────────────────────── */
static void prompt(void)
{
    putStr("> ");
}

static void banner(void)
{
    putStr("\r\n*** MSP432 Command Shell Ready ***\r\n");
    putStr("Type -help for a list of commands.\r\n\r\n");
    prompt();
}

/* ────────────────  COMMAND IMPLEMENTATIONS  ─────────────────────────── */
static void cmd_about(void)
{
    char msg[160];
    snprintf(msg, sizeof(msg),
             "%s | %s | %s | built %s %s\r\n",
             ABOUT_NAME, ABOUT_ASSIGNMENT, APP_VERSION,
             BUILD_DATE, BUILD_TIME);
    putStr(msg);
}

static void help_detail(const char *topic)
{
    if (!strcmp(topic, "help")) {
        putStr("-help [cmd]   : list all commands or show details for <cmd>\r\n");
    }
    else if (!strcmp(topic, "about")) {
        putStr("-about        : show author, assignment, version, build date/time\r\n");
    }
    else if (!strcmp(topic, "print")) {
        putStr("-print text   : echo text exactly as entered\r\n");
    }
    else if (!strcmp(topic, "memr")) {
        putStr("-memr addrhex : read 32-bit word at <addrhex>\r\n"
               "                (flash 0x00000000-0x0007FFFF | SRAM 0x20000000-0x2007FFFF)\r\n");
    }
    else {
        putStr("No help available for that topic\r\n");
    }
}

static void cmd_help(const char *args)
{
    if (!args || *args == '\0') {
        putStr("Commands: -help  -about  -print  -memr\r\n"
               "Use -help <cmd> for details.\r\n");
    }
    else {
        if (*args == '-') args++;     /* optional leading '-' */
        help_detail(args);
    }
}

static void cmd_print(const char *text)
{
    if (text) putStr(text);
    putStr("\r\n");
}

static bool addrOK(uint32_t a)
{
    return (a < 0x00080000u) ||                   /* flash   0x0000_0000..0x0007_FFFF */
           (a >= 0x20000000u && a < 0x20080000u); /* SRAM    0x2000_0000..0x2007_FFFF */
}

static void cmd_memr(const char *arg)
{
    if (!arg || !*arg) {
        putStr("need address\r\n");
        return;
    }
    {
        uint32_t addr = strtoul(arg, NULL, 16);
        if (!addrOK(addr)) {
            putStr("addr out of range\r\n");
            return;
        }
        {
            uint32_t v = *(volatile uint32_t *)addr;
            putHex32(addr); putStr(" : "); putHex32(v); putStr("\r\n");
        }
    }
}

/* ────────────────  PARSER / DISPATCH  ──────────────────────────────── */
static void handleLine(char *line)
{
    char *cmd  = strtok(line, " \t");
    char *args = strtok(NULL, "");

    if (!cmd) return;

    if (*cmd != '-') {
        putStr("?? unknown command (expected leading '-')\r\n");
        return;
    }
    cmd++;

    if (args && *args == '-') args++;

    if      (!strcmp(cmd, "help"))  cmd_help(args);
    else if (!strcmp(cmd, "about")) cmd_about();
    else if (!strcmp(cmd, "print")) cmd_print(args);
    else if (!strcmp(cmd, "memr"))  cmd_memr(args);
    else                            putStr("?? unknown command\r\n");
}

/* ────────────────  EDITING HELPERS  ──────────────────────────────── */
static void redrawLine(size_t oldLen)
{
    /* Move to column 0 */
    putStr("\r");

    /* Prompt + current buffer */
    prompt();
    UART_write(gUart, gLineBuf, len);

    /* Clear any leftover characters */
    {
        size_t clearCount = 2 + oldLen; /* prompt is 2 chars */
        size_t i;
        for (i = 0; i < clearCount; i++)
            putChar(' ');
    }

    /* Return & re‑print up to cursor */
    putStr("\r");
    prompt();
    if (cursor > 0)
        UART_write(gUart, gLineBuf, cursor);
}

static void deleteAtCursor(void)
{
    if (cursor == 0) return;
    {
        size_t oldLen = len;
        size_t i;
        for (i = cursor - 1; i < len - 1; i++)
            gLineBuf[i] = gLineBuf[i + 1];
        len--; cursor--;
        redrawLine(oldLen);
    }
}

static void killLine(void)
{
    size_t oldLen = len;
    len = cursor = 0;
    redrawLine(oldLen);
}

/* ────────────────  MAIN SHELL TASK  ───────────────────────────────── */
void *mainThread(void *arg0)
{
    /* Board + UART init */
    GPIO_init();
    UART_init();
    GPIO_write(CONFIG_GPIO_LED_0, CONFIG_GPIO_LED_ON);

    UART_Params p;
    UART_Params_init(&p);
    p.baudRate       = 115200;
    p.readDataMode   = UART_DATA_BINARY;
    p.writeDataMode  = UART_DATA_BINARY;
    p.readReturnMode = UART_RETURN_FULL;

    gUart = UART_open(CONFIG_UART_0, &p);
    if (!gUart) while (1);

    banner();

    {
        char ch;
        for (;;) {
            UART_read(gUart, &ch, 1);

            /* ===== Newline ===== */
            if (ch == '\r' || ch == '\n') {
                putStr("\r\n");
                if (len > 0) {
                    strncpy(history, gLineBuf, len);
                    history[len] = '\0';
                    hasHistory = true;
                    gLineBuf[len] = '\0';
                    handleLine(gLineBuf);
                }
                len = cursor = 0;
                prompt();
                continue;
            }

            /* ===== Backspace/Delete ===== */
            if (ch == 0x08 || ch == 0x7F) { deleteAtCursor(); continue; }

            /* ===== Ctrl‑U ===== */
            if (ch == 0x15) { killLine(); continue; }

            /* ===== Escape sequences (arrows) ===== */
            if (ch == 0x1B) {
                char s1, s2;
                UART_read(gUart, &s1, 1);
                UART_read(gUart, &s2, 1);
                if (s1 == '[') {
                    if (s2 == 'A') {
                        if (hasHistory) {
                            size_t oldLen = len;
                            strcpy(gLineBuf, history);
                            len = cursor = strlen(history);
                            redrawLine(oldLen);
                        }
                    }
                    else if (s2 == 'B') {
                        size_t oldLen = len;
                        len = cursor = 0;
                        redrawLine(oldLen);
                    }
                    else if (s2 == 'C') {
                        if (cursor < len) {
                            putChar(gLineBuf[cursor]);
                            cursor++;
                        }
                    }
                    else if (s2 == 'D') {
                        if (cursor > 0) {
                            putStr("\b");
                            cursor--;
                        }
                    }
                }
                continue;
            }

            /* ===== Printable chars ===== */
            if (isprint((unsigned char)ch)) {
                if (len < MAX_CMD_LEN - 1) {
                    if (cursor < len) {
                        size_t oldLen = len;
                        size_t i;
                        for (i = len; i > cursor; i--) gLineBuf[i] = gLineBuf[i - 1];
                        gLineBuf[cursor] = ch;
                        len++; cursor++;
                        redrawLine(oldLen);
                    }
                    else {
                        gLineBuf[len++] = ch;
                        cursor = len;
                        putChar(ch);
                    }
                }
                else {
                    putStr("\r\n!! character-overflow (31 max) – start again\r\n");
                    len = cursor = 0;
                    prompt();
                }
            }
            /* other control chars ignored */
        }
    }
}
