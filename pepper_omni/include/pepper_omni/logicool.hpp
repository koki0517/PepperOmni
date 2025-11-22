#pragma once

enum class Buttons    // 右はWebGUI
{
    A,
    B,
    X,
    Y,
    LB,
    RB,
    BACK,
    START,
    UNKNOWN,
    L_PUSH,
    R_PUSH,
    UNKNOWN2
};

enum class Axes
{
    LX,
    LY,
    LT,
    RX,
    RY,
    RT,
    BTN_X,
    BTN_Y
};

/*

msg->axes

LX:     左スティックx座標　左が-1.0、右が1.0
LY:     左スティックy座標　上が1.0、下が-1.0
LT:     L2　何も押さないが1.0、押し込みが-1.0
RX:     右スティックx座標　左が-1.0、右が1.0
RY:     右スティックy座標　上が1.0、下が-1.0
RT:     R2　何も押さないが1.0、押し込みが-1.0
BTN_X:  十字キー　左が-1.0、右が1.0
BTN_Y:  十字キー　上が1.0、下が-1.0

*/