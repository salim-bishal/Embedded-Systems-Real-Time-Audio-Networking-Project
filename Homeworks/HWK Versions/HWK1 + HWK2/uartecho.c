/*
 *  MSP432E401Y Enhanced Command-Line Shell (Redraw-Fixed)  ──  v1.6.2+fix
 *  ────────────────────────────────────────────────────────────────────────
 *  Features:
 *    • Prompt "> " after banner and after each command
 *    • Line editing:
 *        – Backspace/Delete deletes at cursor (shifts remainder left, then redraws)
 *        – Ctrl-U clears entire line
 *        – Left/Right arrows move the cursor within the line
 *        – Up/Down arrows recall or clear single-entry history
 *    • Hard overflow at 31 printable chars:
 *        – Immediately warns, clears buffer, new prompt
 *    • Commands (no leading hyphens):
 *        help [cmd]   : overview or detailed help
 *        about        : author/assignment/version/build info
 *        print <text> : echo text verbatim
 *        memr <hex>   : read 32-bit word from hex address (flash or SRAM)
 *
 *  Build environment: CCS 12.8.0, SimpleLink MSP432E4 SDK 4.x
 *
 *  This version avoids wrapping when editing long lines by clearing exactly
 *  (prompt + oldLen) spaces, not a fixed 64 spaces.
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

/* ────────────────  METADATA  (UPDATE AS NEEDED)  ─────────────── */
#define ABOUT_NAME        "Salim Sadman Bishal"
#define ABOUT_ASSIGNMENT  "ECE 5380 HWK1"
#define APP_VERSION       "v1.6.2"
#define BUILD_DATE        __DATE__
#define BUILD_TIME        __TIME__
/* ─────────────────────────────────────────────────────────────────────── */

#define RX_BUF_SZ   64    /* total buffer size (includes headroom) */
#define MAX_CMD_LEN 32    /* 31 printable chars + 1 NUL */

/* ────────────────  FILE-SCOPE STATE  ────────────────────────────── */
static UART_Handle gUart;                       // UART handle returned by UART_open()
static char       gLineBuf[RX_BUF_SZ];          // buffer for current input line
static size_t     len = 0;                      // number of valid chars in gLineBuf (0..31)
static size_t     cursor = 0;                   // cursor position within [0..len]

/* Single-entry history */
static char       history[RX_BUF_SZ] = {0};
static bool       hasHistory = false;

/* ────────────────  UTILITY I/O HELPERS  ────────────────────────── */
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

/* ────────────────  PROMPT & BANNER  ─────────────────────────────── */
static void prompt(void)
{
    putStr("> ");
}

static void banner(void)
{
    putStr("\r\n*** MSP432 Command Shell Ready ***\r\n");
    putStr("Type help for a list of commands.\r\n\r\n");
    prompt();
}

/* ────────────────  COMMAND IMPLEMENTATIONS  ────────────────────── */
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
        putStr("help [cmd]   : list all commands or show details for <cmd>\r\n");
    }
    else if (!strcmp(topic, "about")) {
        putStr("about        : show author, assignment, version, build date/time\r\n");
    }
    else if (!strcmp(topic, "print")) {
        putStr("print text   : echo text exactly as entered\r\n");
    }
    else if (!strcmp(topic, "memr")) {
        putStr("memr addrhex : read 32-bit word at hex address (flash/SRAM)\r\n");
    }
    else {
        putStr("No help available for that topic\r\n");
    }
}

static void cmd_help(const char *args)
{
    if (!args || *args == '\0') {
        putStr("Commands: help  about  print  memr\r\n"
               "Use help <cmd> for details.\r\n");
    }
    else {
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
    uint32_t addr = strtoul(arg, NULL, 16);
    if (!addrOK(addr)) {
        putStr("addr out of range\r\n");
        return;
    }
    uint32_t v = *(volatile uint32_t *)addr;
    putHex32(addr); putStr(" : "); putHex32(v); putStr("\r\n");
}

/* ────────────────  PARSER / DISPATCH  ──────────────────────────── */
static void handleLine(char *line)
{
    char *cmd  = strtok(line, " \t");
    char *args = strtok(NULL, "");

    if (!cmd) return;

    if      (!strcmp(cmd, "help"))  cmd_help(args);
    else if (!strcmp(cmd, "about")) cmd_about();
    else if (!strcmp(cmd, "print")) cmd_print(args);
    else if (!strcmp(cmd, "memr"))  cmd_memr(args);
    else                            putStr("?? unknown command\r\n");
}

/* ────────────────  EDITING HELPERS  ────────────────────────────── */

/**
 * redrawLine:
 *   - Clears exactly (prompt + oldLen) columns, then re-prints prompt + buffer up to cursor.
 *   - “oldLen” is the length of gLineBuf before editing, so any leftover chars
 *     from a previous longer line get erased without wrapping.
 */
static void redrawLine(size_t oldLen)
{
    // 1) Carriage return to start of line
    putStr("\r");

    // 2) Print prompt and current buffer
    prompt();
    UART_write(gUart, gLineBuf, len);

    // 3) Overwrite exactly (2 + oldLen) spaces
    {
        size_t clearCount = 2 + oldLen;
        size_t i;
        for (i = 0; i < clearCount; i++) {
            putChar(' ');
        }
    }

    // 4) Carriage return again
    putStr("\r");

    // 5) Print prompt + buffer up to cursor to position cursor correctly
    prompt();
    if (cursor > 0) {
        UART_write(gUart, gLineBuf, cursor);
    }
}

/**
 * deleteAtCursor:
 *   - Deletes the character immediately to the left of the cursor (if any),
 *     shifts the tail left by one, decrements len & cursor, then redraws.
 */
static void deleteAtCursor(void)
{
    if (cursor == 0) {
        // Nothing to delete
        return;
    }
    size_t oldLen = len;
    {
        size_t i;
        for (i = cursor - 1; i < len - 1; i++) {
            gLineBuf[i] = gLineBuf[i + 1];
        }
    }
    len--;
    cursor--;
    redrawLine(oldLen);
}

/**
 * killLine:
 *   - Bound to Ctrl-U (ASCII 0x15). Clears the entire current line.
 *   - Simply sets len=cursor=0 and redraws (oldLen was whatever it was).
 */
static void killLine(void)
{
    size_t oldLen = len;
    len = 0;
    cursor = 0;
    redrawLine(oldLen);
}

/* ────────────────  MAIN SHELL TASK  ────────────────────────────── */
void *mainThread(void *arg0)
{
    /* 1) Board and UART initialization */
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
    if (!gUart) {
        while (1);  // fatal error if UART won't open
    }

    /* 2) Print banner + prompt */
    banner();

    /* 3) Infinite receive-loop: read one byte at a time */
    char ch;
    for (;;)
    {
        UART_read(gUart, &ch, 1);

        /* ===== 3.1 Newline (Enter) Handling ===== */
        if (ch == '\r' || ch == '\n') {
            putStr("\r\n");
            if (len > 0) {
                // Save this line to history
                strncpy(history, gLineBuf, len);
                history[len] = '\0';
                hasHistory = true;

                // Process the command
                gLineBuf[len] = '\0';
                handleLine(gLineBuf);
            }
            // Clear buffer for next command
            len = 0;
            cursor = 0;
            prompt();
            continue;
        }

        /* ===== 3.2 Backspace/Delete Handling ===== */
        if (ch == 0x08 || ch == 0x7F) {
            deleteAtCursor();
            continue;
        }

        /* ===== 3.3 Ctrl-U (kill entire line) ===== */
        if (ch == 0x15) {  // ASCII 0x15 = Ctrl-U
            killLine();
            continue;
        }

        /* ===== 3.4 Arrow-Key & History Handling ===== */
        if (ch == 0x1B) {  // ASCII 0x1B = ESC
            char seq1, seq2;
            UART_read(gUart, &seq1, 1);
            UART_read(gUart, &seq2, 1);
            if (seq1 == '[') {
                if (seq2 == 'A') {
                    /* Up arrow: recall history (if available) */
                    if (hasHistory) {
                        size_t oldLen = len;
                        strcpy(gLineBuf, history);
                        len = strlen(history);
                        cursor = len;
                        redrawLine(oldLen);
                    }
                }
                else if (seq2 == 'B') {
                    /* Down arrow: clear current line */
                    size_t oldLen = len;
                    len = 0;
                    cursor = 0;
                    redrawLine(oldLen);
                }
                else if (seq2 == 'C') {
                    /* Right arrow: move cursor right (if possible) */
                    if (cursor < len) {
                        putChar(gLineBuf[cursor]);
                        cursor++;
                    }
                }
                else if (seq2 == 'D') {
                    /* Left arrow: move cursor left (if possible) */
                    if (cursor > 0) {
                        putStr("\b");
                        cursor--;
                    }
                }
            }
            continue;
        }

        /* ===== 3.5 Printable Characters ===== */
        if (isprint((unsigned char)ch)) {
            if (len < MAX_CMD_LEN - 1) {
                if (cursor < len) {
                    /* Insert in middle: shift tail right */
                    size_t oldLen = len;
                    {
                        size_t i;
                        for (i = len; i > cursor; i--) {
                            gLineBuf[i] = gLineBuf[i - 1];
                        }
                    }
                    gLineBuf[cursor] = ch;
                    len++;
                    cursor++;
                    redrawLine(oldLen);
                }
                else {
                    /* Append at end */
                    gLineBuf[len++] = ch;
                    cursor = len;
                    putChar(ch);
                }
            }
            else {
                /* Overflow at 31 chars */
                putStr("\r\n!! character-overflow (31 max) – start again\r\n");
                len = 0;
                cursor = 0;
                prompt();
            }
        }
        /* ===== 3.6 Any other control chars: ignored ===== */
    }
}
