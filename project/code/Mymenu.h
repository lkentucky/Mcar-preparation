#ifndef MCAR_MY_MENU_H
#define MCAR_MY_MENU_H
void Menu_Init(void);
void Menu_Show(void);
void Menu_Switch(void);
/* 与原 20ms 按键扫描一起调用，每 100ms 请求显示刷新。 */
void Menu_Tick_20ms(void);
#endif
