#ifndef MCAR_MY_MENU_H
#define MCAR_MY_MENU_H

void Menu_Init(void);
void Menu_Show(void);
void Menu_Switch(void);
/* Called with the existing 20 ms key scan, to refresh live measurements. */
void Menu_Tick_20ms(void);

#endif
