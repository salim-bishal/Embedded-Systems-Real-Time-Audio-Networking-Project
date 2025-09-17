/*
 *  MSP432E401Y Command Line Shell   ──  v1.6.0
 *  ────────────────────────────────────────────────────────────────
 *  Features
 *    • Prompt ">" after banner and after each command
 *    • Line editing: Backspace, Ctrl-U (kill line)
 *    • Hard overflow at 31 printable chars
 *        – warning is printed, buffer cleared, new prompt shown
 *    • Commands (no leading hyphen):
 *        help   [cmd]        overview or per‑command help
 *        about              author / assignment / version
 *        print  <text>      echo text verbatim
 *        memr   <hex addr>  read 32‑bit word at address
 *
 *  Build environment: CCS 12.8.0, SimpleLink MSP432E4 SDK 4.x
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

/* ────────────────  METADATA  (EDIT PER SUBMISSION)  ─────────────── */
#define ABOUT_NAME        "Salim Sadman Bishal"   /* ← CHANGE HERE          */
#define ABOUT_ASSIGNMENT  "ECE 5380 HWK1"         /* ← CHANGE HERE          */
#define APP_VERSION       "v1.6.0"               /* bump each new feature */
#define BUILD_DATE        __DATE__
#define BUILD_TIME        __TIME__
/* ─────────────────────────────────────────────────────────────────── */

#define RX_BUF_SZ   64          /* size of receive buffer             */
#define MAX_CMD_LEN 32          /* 31 chars + '\0'                    */

/* ────────────────  FILE-SCOPE STATE  ────────────────────────────── */
static UART_Handle gUart;
static char   gLineBuf[RX_BUF_SZ];
static size_t idx = 0;          /* current length of gLineBuf        */

/* ────────────────  UTILITY I/O  ─────────────────────────────────── */
static void putStr(const char *s)           { UART_write(gUart, s, strlen(s)); }
static void putChar(char c)                 { UART_write(gUart, &c, 1); }

static void putHex32(uint32_t v)            /* prints 0xXXXXXXXX        */
{
    char buf[11];
    snprintf(buf, sizeof buf, "0x%08X", v);
    putStr(buf);
}

/* ────────────────  PROMPT & BANNER  ─────────────────────────────── */
static void prompt(void)  { putStr("> "); }  /* shown after every line */

static void banner(void)
{
    putStr("\r\n*** MSP432 Command Shell Ready ***\r\n"
           "Type help for a list of commands.\r\n\r\n");
    prompt();
}

/* ────────────────  COMMAND IMPLEMENTATIONS  ────────────────────── */
static void cmd_about(void)
{
    char msg[160];
    snprintf(msg, sizeof msg,
             "%s | %s | %s | built %s %s\r\n",
             ABOUT_NAME, ABOUT_ASSIGNMENT, APP_VERSION,
             BUILD_DATE, BUILD_TIME);
    putStr(msg);
}

/* detailed help for each command */
static void help_detail(const char *topic)
{
    if      (!strcmp(topic, "help"))
        putStr("help [cmd]   : show list or explain <cmd>\r\n");
    else if (!strcmp(topic, "about"))
        putStr("about        : show author, assignment, version\r\n");
    else if (!strcmp(topic, "print"))
        putStr("print text   : echo text exactly as entered\r\n");
    else if (!strcmp(topic, "memr"))
        putStr("memr addrhex : read 32-bit word at hex addr\r\n");
    else
        putStr("No help available for that topic\r\n");
}

static void cmd_help(const char *args)
{
    if (!args || *args == '\0')
        putStr("Commands: help  about  print  memr\r\n"
               "Use help <cmd> for details.\r\n");
    else
        help_detail(args);
}

static void cmd_print(const char *text)
{
    if (text) putStr(text);
    putStr("\r\n");
}

/* simple flash+SRAM range check for MSP432E401Y */
static bool addrOK(uint32_t a)
{
    return (a < 0x00080000u) || (a >= 0x20000000u && a < 0x20080000u);
}

static void cmd_memr(const char *arg)
{
    if (!arg || !*arg) { putStr("need address\r\n"); return; }

    uint32_t addr = strtoul(arg, NULL, 16);
    if (!addrOK(addr)) { putStr("addr out of range\r\n"); return; }

    uint32_t v = *(volatile uint32_t *)addr;
    putHex32(addr); putStr(" : "); putHex32(v); putStr("\r\n");
}

/* ────────────────  PARSER / DISPATCH  ──────────────────────────── */
static void handleLine(char *line)
{
    /* split into first token and the rest */
    char *cmd  = strtok(line, " \t");
    char *args = strtok(NULL,  "");    /* may be NULL */

    if (!cmd) return;

    if      (!strcmp(cmd, "help"))  cmd_help(args);
    else if (!strcmp(cmd, "about")) cmd_about();
    else if (!strcmp(cmd, "print")) cmd_print(args);
    else if (!strcmp(cmd, "memr"))  cmd_memr(args);
    else                            putStr("?? unknown command\r\n");
}

/* ────────────────  EDITING HELPERS  ────────────────────────────── */
static void resetLine(void) { idx = 0; }           /* clear buffer */

static void backspace(void)
{
    if (idx) {
        idx--;
        putStr("\b \b");                /* erase last char visually */
    }
}

static void killLine(void)               /* Ctrl‑U support */
{
    while (idx) backspace();
}

/* ────────────────  MAIN SHELL TASK  ────────────────────────────── */
void *mainThread(void *arg0)
{
    /* Basic board & UART init */
    GPIO_init(); UART_init();
    GPIO_write(CONFIG_GPIO_LED_0, CONFIG_GPIO_LED_ON);

    UART_Params p; UART_Params_init(&p);
    p.baudRate = 115200;
    p.readDataMode  = UART_DATA_BINARY;
    p.writeDataMode = UART_DATA_BINARY;
    p.readReturnMode = UART_RETURN_FULL;
    gUart = UART_open(CONFIG_UART_0, &p);
    if (!gUart) while (1);              /* fatal */

    banner();                           /* greeting + first prompt */

    char ch;
    for (;;)
    {
        UART_read(gUart, &ch, 1);

        /* ───── Newline received ─────────────────────────────── */
        if (ch == '\r' || ch == '\n') {
            putStr("\r\n");             /* move to next line */

            if (idx) {                  /* non-empty command */
                gLineBuf[idx] = '\0';
                handleLine(gLineBuf);
            }
            resetLine();
            prompt();
            continue;
        }

        /* ───── Backspace / Delete ──────────────────────────── */
        if (ch == 0x08 || ch == 0x7F) { backspace(); continue; }

        /* ───── Ctrl‑U kills the entire line ────────────────── */
        if (ch == 0x15) { killLine(); continue; }

        /* ───── Printable bytes ─────────────────────────────── */
        if (isprint((unsigned char)ch))
        {
            if (idx < MAX_CMD_LEN - 1) {
                gLineBuf[idx++] = ch;
                putChar(ch);            /* local echo */
            } else {
                /* overflow: warn, flush, new prompt */
                putStr("\r\n!! character overflow (31 max) – start again\r\n");
                resetLine();
                prompt();
            }
        }
        /* all other control chars are ignored */
    }
}
