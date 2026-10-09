/** @file licor_shell.h @brief Foreground LI-COR shell commands. */
#ifndef LICOR_SHELL_H
#define LICOR_SHELL_H

void cmd_licor(int argc, char **argv);
void cmd_licor_raw(int argc, char **argv);
void cmd_licor_info(int argc, char **argv);
void cmd_licor_discover(int argc, char **argv);
void cmd_licor_model(int argc, char **argv);
void cmd_licor_configure(int argc, char **argv);
void cmd_licor_query(int argc, char **argv);
void cmd_licor_set(int argc, char **argv);
void cmd_licor_heater(int argc, char **argv);
void cmd_licor_resync(int argc, char **argv);

#endif
