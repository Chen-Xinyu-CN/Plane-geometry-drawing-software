#include <windows.h>
#include <math.h>
#include <iostream>
#include <graphics.h>
#include <vector>
#include <stack>
#define _CRT_SECURE_NO_WARNINGS
using namespace std;
int width = 1200, height = 900, color = 255;
int chosen_switch = 0, jsq = 0, bse = 0, Are_count = 0, f_select = 1, g_select = 1;
int white = 255 * 256 * 256 + 255 * 256 + 255;
int DiceValue; int st;
int IfDiceUsed;
const int INSIDE = 0; // Both endpoints are inside the clip window
const int LEFT = 1;   // Bit 0 is set if the point is left of the clip window
const int RIGHT = 2;  // Bit 1 is set if the point is right of the clip window
const int BOTTOM = 4; // Bit 2 is set if the point is below the clip window
const int TOP = 8;    // Bit 3 is set if the point is above the clip window
int window[1201][901] = { 0 };
int BZlabel;
struct chesscub {
    int x = 0;
    int y = 0;
    int col;
    int lab = -1;
    int plant = 0;
};
chesscub board[92];
chesscub board_e[4][9];
chesscub start[4][4];

int startx, starty;
int step = 50;
void change_color() {
    if (color < 255 * 256 * 256) {
        if (color == 0) {
            color = 255;
        }
        else color *= 256;
    }
    else {
        color = 0;
    }
}

// 全局变量，用于保存当前绘制操作类型
enum DrawType {
    DRAW_LINE,
    DRAW_Beize,
    DRAW_LINE_Bre,
    DRAW_LINE_Mid,
    DRAW_CIRCLE,
    DRAW_Are,
    DRAW_RECTANGLE,
    DRAW_DBX,
    CUT_LINE_MID,
    CUT_LINE_CS,
    FILL,
    CHOOSE,
    MOVE,
    ADD,
    DISADD,
    ROLL,
    BP,
    game,
    gamedice,
    CHANGE_BZ
};
// 线型类型枚举
enum LineStyle {
    LINE_SOLID,
    LINE_DASH,
    LINE_DOT,
};
// 线型线宽属性结构体
struct LineAttributes {
    int width;
    LineStyle style;
};
struct p {
    int x;
    int y;
};
struct gragh_information {
    DrawType lab;
    LineAttributes XianXin;
    int live;
    int choosing = 0;
    int pos = 0;
    int color;
};
class graph {

public:
    int sum = -1;
    vector<gragh_information>gragh_inf;
    vector<vector<p>>point;
    vector<p>temp;
    int currentSelect = -1;
    void insert_i(DrawType lab, LineAttributes x, int pos = 0, int color = color) {
        gragh_information tem;
        tem.live = 1;
        tem.lab = lab;
        tem.XianXin = x,
            tem.pos = pos;
        tem.color = color;
        gragh_inf.push_back(tem);
    }
    void insert_p(int x, int y) {
        p tem;
        tem.x = x; tem.y = y;
        temp.push_back(tem);
    }
    void insert_g() {
        point.push_back(temp);
        temp.clear();
        sum++;
    }


};
graph screan;
LineAttributes currentLineAttributes = { 1, LINE_SOLID }; // 初始线宽为1，实线
DrawType currentDrawType = DRAW_CIRCLE;
// 绘制函数

int DrawPoint_count = 0; int DrawPoint_bool = 1;
void DrawPoint_P(HDC hdc, int x, int y, int color) {
    int d = currentLineAttributes.width;
    for (int i = 0; i <= d; i++) {
        for (int j = y - d + i; j <= y + d - i; j++) {
            //SetPixel(hdc, x - i, j, color);
            //SetPixel(hdc, x + i, j, color);
            if (j <= 900 && j >= 0) {
                if (x - i <= 1200 && x - i >= 0) {
                    window[x - i][j] = color;
                }
                if (x - i <= 1200 && x - i >= 0) {
                    window[x + i][j] = color;
                }
            }

        }
    }
}
void DrawPoint(HDC hdc, int x, int y, COLORREF) {
    if (currentLineAttributes.style == LINE_SOLID) {
        DrawPoint_P(hdc, x, y, color);

    }
    else {
        int mod = currentLineAttributes.width;
        if (currentLineAttributes.style == LINE_DASH) {
            mod *= 10;
            if (DrawPoint_count > mod) {
                DrawPoint_count -= mod;
                DrawPoint_bool = 1 - DrawPoint_bool;
            }
            if (DrawPoint_bool) {
                DrawPoint_P(hdc, x, y, color);
            }
        }
        if (currentLineAttributes.style == LINE_DOT) {
            mod *= 4;
            if (DrawPoint_count > mod) {
                DrawPoint_count -= mod;
                DrawPoint_bool = 1;
            }
            if (DrawPoint_bool) {
                DrawPoint_bool = 0;
                DrawPoint_P(hdc, x, y, color);
            }
        }
        DrawPoint_count++;
    }
}
double CalculateAngle(int xCenter, int yCenter, int xPoint, int yPoint) {
    // 计算相对于圆心的点的坐标差
    double deltaX = static_cast<double>(xPoint - xCenter);
    double deltaY = static_cast<double>(yPoint - yCenter);

    // 使用 atan2 计算角度（以弧度为单位）
    double angleRad = atan2(deltaY, deltaX);

    // 将弧度转换为度
    double angleDeg = angleRad * 180.0 / 3.14159265359;

    // 确保角度在[0, 360)范围内
    if (angleDeg < 0.0) {
        angleDeg += 360.0;
    }

    return angleDeg;
}
double CalculateR(int xCenter, int yCenter, int xPoint, int yPoint) {
    // 计算相对于圆心的点的坐标差
    double R = sqrt((xCenter - xPoint) * (xCenter - xPoint) + (yCenter - yPoint) * (yCenter - yPoint));

    return R;
}
void Bre(HDC hdc, int x0, int y0, int x1, int y1, int color) {
    bool steep = false;

    // 如果斜率的绝对值大于1，交换 x 和 y 坐标，使斜率在[-1, 1]范围内
    if (abs(x1 - x0) < abs(y1 - y0)) {
        std::swap(x0, y0);
        std::swap(x1, y1);
        steep = true;
    }
    if (x0 > x1) {
        std::swap(x0, x1);
        std::swap(y0, y1);
    }

    int dx = x1 - x0;
    int dy = abs(y1 - y0);
    int yStep = (y0 < y1) ? 1 : -1;
    int y = y0;
    int error = dx / 2;

    for (int x = x0; x <= x1; x++) {
        if (steep) {
            DrawPoint(hdc, y, x, color);
        }
        else {
            DrawPoint(hdc, x, y, color);
        }
        error -= dy;
        if (error < 0) {
            y += yStep;
            error += dx;
        }
    }
}
void Mid(HDC hdc, int x0, int y0, int x1, int y1, int color) {
    bool steep = false;
    // 检查斜率绝对值是否大于1，如果是，交换 x 和 y 坐标，并标记为 steep
    if (abs(x1 - x0) < abs(y1 - y0)) {
        std::swap(x0, y0);
        std::swap(x1, y1);
        steep = true;
    }
    if (x0 > x1) {
        std::swap(x0, x1);
        std::swap(y0, y1);
    }
    int dx = abs(x1 - x0);
    int dy = abs(y1 - y0);
    int yStep = (y0 < y1) ? 1 : -1;
    int error = dx / 2;
    int y = y0;

    for (int x = x0; x <= x1; x++) {
        if (steep) {
            DrawPoint(hdc, y, x, color);
        }
        else {
            DrawPoint(hdc, x, y, color);
        }

        error -= dy;
        if (error < 0) {
            y += yStep;
            error += dx;
        }
    }
}
void DrawLine(HDC hdc, int startX, int startY, int endX, int endY) {
    Bre(hdc, startX, startY, endX, endY, color);
}
void DrawLine_Mid(HDC hdc, int startX, int startY, int endX, int endY) {
    Mid(hdc, startX, startY, endX, endY, color);
}
void DrawArc(HDC hdc, int xCenter, int yCenter, int radius, float startAngle, float endAngle, COLORREF color) {
    // 将角度转换为弧度
    startAngle = startAngle * 3.14159265359 / 180.0;
    endAngle = endAngle * 3.14159265359 / 180.0;
    if (startAngle > endAngle)startAngle -= 3.14159265359 * 2;
    for (float angle = startAngle; angle < endAngle; angle += 0.0001) {
        if (angle >= 3.14159265359 * 2) {
            angle -= 3.14159265359 * 2;
            break;
        }
        int x = xCenter + radius * cos(angle);
        int y = yCenter + radius * sin(angle);
        DrawPoint(hdc, x, y, color);
    }
}
void Draw_Circle(HDC hdc, int xCenter, int yCenter, int radius, COLORREF color) {
    int x = radius;
    int y = 0;
    int error = 1 - radius;
    DrawPoint(hdc, xCenter, yCenter, color);
    DrawPoint_count = 0;
    DrawPoint_bool = 1;
    int f, ff;
    while (x >= y) {
        f = DrawPoint_bool; ff = DrawPoint_count;
        DrawPoint(hdc, xCenter + x, yCenter + y, color);
        DrawPoint_count = ff; DrawPoint_bool = f;
        DrawPoint(hdc, xCenter - x, yCenter + y, color);
        DrawPoint_count = ff; DrawPoint_bool = f;
        DrawPoint(hdc, xCenter + x, yCenter - y, color);
        DrawPoint_count = ff; DrawPoint_bool = f;
        DrawPoint(hdc, xCenter - x, yCenter - y, color);
        DrawPoint_count = ff; DrawPoint_bool = f;
        DrawPoint(hdc, xCenter + y, yCenter + x, color);
        DrawPoint_count = ff; DrawPoint_bool = f;
        DrawPoint(hdc, xCenter - y, yCenter + x, color);
        DrawPoint_count = ff; DrawPoint_bool = f;
        DrawPoint(hdc, xCenter + y, yCenter - x, color);
        DrawPoint_count = ff; DrawPoint_bool = f;
        DrawPoint(hdc, xCenter - y, yCenter - x, color);

        if (error <= 0) {
            y++;
            error += 2 * y + 1;
        }
        if (error > 0) {
            x--;
            error -= 2 * x + 1;
        }
    }
}
void MidPointCircle(HDC hdc, int xCenter, int yCenter, int endX, int endY) {
    double  radius;
    radius = sqrt((endX - xCenter) * (endX - xCenter) + (endY - yCenter) * (endY - yCenter));
    Draw_Circle(hdc, xCenter, yCenter, radius, color);
}
int findNearestShape(int x, int y) {
    double minDistance = INT_MAX;
    int nearestShapeIndex = -1;

    for (int i = 0; i < screan.point.size(); i++) {
        if (screan.gragh_inf[i].lab == DRAW_CIRCLE || screan.gragh_inf[i].lab == DRAW_Are) {
            double distance = sqrt(pow(screan.point[i][0].x - x, 2) + pow(screan.point[i][0].y - y, 2));
            if (distance < minDistance) {
                minDistance = distance;
                nearestShapeIndex = i;
            }
        }
        else {
            for (int j = 0; j < screan.point[i].size(); j++) {
                double distance = sqrt(pow(screan.point[i][j].x - x, 2) + pow(screan.point[i][j].y - y, 2));
                if (distance < minDistance) {
                    minDistance = distance;
                    nearestShapeIndex = i;
                    BZlabel = j;
                }
            }
        }
    }
    return nearestShapeIndex;
}
int ComputeOutCode(int x, int y, int xMin, int yMin, int xMax, int yMax) {
    int code = 0;

    if (x < xMin) {
        code |= 1; // 设置第1位表示左边界
    }
    else if (x > xMax) {
        code |= 2; // 设置第2位表示右边界
    }

    if (y < yMin) {
        code |= 4; // 设置第3位表示底边界
    }
    else if (y > yMax) {
        code |= 8; // 设置第4位表示顶边界
    }

    return code;
}
void ClipMidPoint(int& x, int& y, int& xOther, int& yOther, int xMin, int yMin, int xMax, int yMax, int outCode) {
    // 确定线段与裁剪窗口的交点
    if (outCode & 1) {
        // 线段与裁剪窗口左边界相交
        // 计算交点的y坐标
        y = y + (yOther - y) * (xMin - x) / (xOther - x);
        x = xMin; // 更新x坐标为裁剪窗口左边界
    }
    else if (outCode & 2) {
        // 线段与裁剪窗口右边界相交
        // 计算交点的y坐标
        y = y + (yOther - y) * (xMax - x) / (xOther - x);
        x = xMax; // 更新x坐标为裁剪窗口右边界
    }
    else if (outCode & 4) {
        // 线段与裁剪窗口底边界相交
        // 计算交点的x坐标
        x = x + (xOther - x) * (yMin - y) / (yOther - y);
        y = yMin; // 更新y坐标为裁剪窗口底边界
    }
    else if (outCode & 8) {
        // 线段与裁剪窗口顶边界相交
        // 计算交点的x坐标
        x = x + (xOther - x) * (yMax - y) / (yOther - y);
        y = yMax; // 更新y坐标为裁剪窗口顶边界
    }
}
void ClipMidPointSubdivision(HDC hdc, int& x0, int& y0, int& x1, int& y1, int xMin, int yMin, int xMax, int yMax) {
    bool isInside = false;
    bool isOutside = false;

    while (true) {
        int outCode0 = ComputeOutCode(x0, y0, xMin, yMin, xMax, yMax);
        int outCode1 = ComputeOutCode(x1, y1, xMin, yMin, xMax, yMax);

        if ((outCode0 | outCode1) == 0) {
            // 线段完全在裁剪窗口内
            isInside = true;
            break;
        }
        else if (outCode0 & outCode1) {
            // 线段完全在裁剪窗口外
            isOutside = true;
            break;
        }
        else {
            // 线段部分在裁剪窗口内，需要进行裁剪
            int tempcolor = color;
            color = white;
            DrawLine_Mid(hdc, x0, y0, x1, y1);
            if (outCode0 != 0) {
                ClipMidPoint(x0, y0, x1, y1, xMin, yMin, xMax, yMax, outCode0);
            }
            if (outCode1 != 0) {
                ClipMidPoint(x1, y1, x0, y0, xMin, yMin, xMax, yMax, outCode1);
            }
            color = tempcolor;
            DrawLine(hdc, x0, y0, x1, y1);
        }
    }

    if (isInside) {
        // 如果线段完全在裁剪窗口内，则不操作线段
        return;
    }
    else if (isOutside) {
        // 线段完全在裁剪窗口外，清空线段
        int tempcolor = color;
        color = white;
        DrawLine_Mid(hdc, x0, y0, x1, y1);
        return;
    }
}
void ClipMidPointSubdivision(HDC hdc, int xMin, int yMin, int xMax, int yMax) {
    for (int i = 0; i < screan.point.size(); i++) {
        if (screan.gragh_inf[i].lab == DRAW_LINE || screan.gragh_inf[i].lab == DRAW_LINE_Bre || screan.gragh_inf[i].lab == DRAW_LINE_Mid) {
            ClipMidPointSubdivision(hdc, screan.point[i][0].x, screan.point[i][0].y, screan.point[i][1].x, screan.point[i][1].y, xMin, yMin, xMax, yMax);
        }
    }
}
// Calculate the region code for a given point
int ComputeRegionCode(double x, double y, double xMin, double xMax, double yMin, double yMax) {
    const double EPSILON = 1e-10;
    int code = INSIDE;
    if (x < xMin - EPSILON) code |= LEFT;
    if (x > xMax + EPSILON) code |= RIGHT;
    if (y < yMin - EPSILON) code |= BOTTOM;
    if (y > yMax + EPSILON) code |= TOP;
    return code;
}
// Clip a line segment using Cohen-Sutherland algorithm
void CSLineClip(HDC hdc, int& x0, int& y0, int& x1, int& y1, int xMin, int xMax, int yMin, int yMax) {
    int code0, code1, code;
    double dx0 = static_cast<double>(x0),
        dy0 = static_cast<double>(y0),
        dx1 = static_cast<double>(x1),
        dy1 = static_cast<double>(y1),
        dxMin = static_cast<double>(xMin),
        dyMin = static_cast<double>(yMin),
        dxMax = static_cast<double>(xMax),
        dyMax = static_cast<double>(yMax),
        x, y;
    int tempcolor = color;
    color = white;
    DrawLine(hdc, x0, y0, x1, y1);

    code0 = ComputeRegionCode(dx0, dy0, dxMin, dxMax, dyMin, dyMax);
    code1 = ComputeRegionCode(dx1, dy1, dxMin, dxMax, dyMin, dyMax);             // 端点坐标编码
    while (code0 != 0 || code1 != 0)     // 直到”完全可见”
    {
        if ((code0 & code1) != 0) return;  // 排除”显然不可见”情况
        code = code0;
        if (code0 == 0) code = code1;    // 求得在窗口外的点
        //按顺序检测到端点的编码不为0，才把线段与对应的窗口边界求交。
        if ((LEFT & code) != 0)                 // 线段与窗口左边延长线相交
        {
            x = dxMin;
            y = dy0 + (dy1 - dy0) * (dxMin - dx0) / (dx1 - dx0);
        }

        else if ((RIGHT & code) != 0)        // 线段与窗口右边延长线相交
        {
            x = dxMax;
            y = dy0 + (dy1 - dy0) * (dxMax - dx0) / (dx1 - dx0);
        }
        else if ((BOTTOM & code) != 0)     // 线段与窗口下边延长线相交
        {
            y = dyMin;
            x = dx0 + (dx1 - dx0) * (dyMin - dy0) / (dy1 - dy0);
        }
        else if ((TOP & code) != 0)         // 线段与窗口上边延长线相交
        {
            y = dyMax;
            x = dx0 + (dx1 - dx0) * (dyMax - dy0) / (dy1 - dy0);
        }
        if (code == code0) {
            dx0 = x;
            dy0 = y;
            code0 = ComputeRegionCode(x, y, xMin, xMax, yMin, yMax);
        } //裁去P1到交点
        else {
            dx1 = x;
            dy1 = y;
            code1 = ComputeRegionCode(x, y, xMin, xMax, yMin, yMax);
        }                     //裁去P2到交点
    }
    color = tempcolor;
    x0 = static_cast<int>(dx0 + 0.5);
    y0 = static_cast<int>(dy0 + 0.5);
    x1 = static_cast<int>(dx1 + 0.5);
    y1 = static_cast<int>(dy1 + 0.5);
    DrawLine(hdc, x0, y0, x1, y1);
}
void CSLineClip(HDC hdc, int xMin, int yMin, int xMax, int yMax) {
    for (int i = 0; i < screan.point.size(); i++) {
        if (screan.gragh_inf[i].lab == DRAW_LINE || screan.gragh_inf[i].lab == DRAW_LINE_Bre || screan.gragh_inf[i].lab == DRAW_LINE_Mid) {
            CSLineClip(hdc, screan.point[i][0].x, screan.point[i][0].y, screan.point[i][1].x, screan.point[i][1].y, xMin, yMin, xMax, yMax);
        }
    }
}
void CohenSutherlandClip(HDC hdc, int& x0, int& y0, int& x1, int& y1, int xMin, int yMin, int xMax, int yMax)
{
    int tempcolor = color;
    color = white;
    DrawLine(hdc, x0, y0, x1, y1);
    int outCode0 = ComputeOutCode(x0, y0, xMin, yMin, xMax, yMax);
    int outCode1 = ComputeOutCode(x1, y1, xMin, yMin, xMax, yMax);

    while ((outCode0 | outCode1) != 0) {
        // 线段完全在裁剪窗口外
        if ((outCode0 & outCode1) != 0) {
            return;
        }

        int x, y;
        int outCodeOut = (outCode0 != 0) ? outCode0 : outCode1;

        // 选择线段与窗口边界相交的点
        if (outCodeOut & 1) {
            x = xMin;
            y = y0 + (y1 - y0) * (xMin - x0) / (x1 - x0);
        }
        else if (outCodeOut & 2) {
            x = xMax;
            y = y0 + (y1 - y0) * (xMax - x0) / (x1 - x0);
        }
        else if (outCodeOut & 4) {
            y = yMin;
            x = x0 + (x1 - x0) * (yMin - y0) / (y1 - y0);
        }
        else if (outCodeOut & 8) {
            y = yMax;
            x = x0 + (x1 - x0) * (yMax - y0) / (y1 - y0);
        }

        // 更新相交点
        if (outCodeOut == outCode0) {
            x0 = x;
            y0 = y;
            outCode0 = ComputeOutCode(x0, y0, xMin, yMin, xMax, yMax);
        }
        else {
            x1 = x;
            y1 = y;
            outCode1 = ComputeOutCode(x1, y1, xMin, yMin, xMax, yMax);
        }
    }

    // 绘制裁剪后的线段
    color = tempcolor;
    DrawLine(hdc, x0, y0, x1, y1);
}
void CohenSutherlandClipAll(HDC hdc, int xMin, int yMin, int xMax, int yMax)
{
    for (int i = 0; i < screan.point.size(); i++) {
        if (screan.gragh_inf[i].lab == DRAW_LINE || screan.gragh_inf[i].lab == DRAW_LINE_Bre || screan.gragh_inf[i].lab == DRAW_LINE_Mid) {
            int x0 = screan.point[i][0].x;
            int y0 = screan.point[i][0].y;
            int x1 = screan.point[i][1].x;
            int y1 = screan.point[i][1].y;

            // 判断线段是否在裁剪窗口内
            if ((x0 >= xMin && x0 <= xMax && y0 >= yMin && y0 <= yMax) || (x1 >= xMin && x1 <= xMax && y1 >= yMin && y1 <= yMax)) {
                CohenSutherlandClip(hdc, x0, y0, x1, y1, xMin, yMin, xMax, yMax);
            }
        }
    }
}
int c = color;
int Dice(HWND hwnd) {
    if (st != 6)change_color();

    std::srand(static_cast<unsigned int>(std::time(nullptr)));
    int t = (std::rand() % 6) + 1;
    int cc = color;
    HDC hdc = GetDC(hwnd);
    for (int i = 0; i < t; i++) {
        Draw_Circle(hdc, step, step * (i + 1), step / 3, color);
        //fillcircle(step, step * (i + 1), step / 2);
    }
    color = white;
    for (int i = t; i < 6; i++) {
        Draw_Circle(hdc, step, step * (i + 1), step / 3, color);
    }
    color = cc;
    change_color();

    return t;

}
int draw_plant(HWND hwnd, int x, int y, int pos) {
    HDC hdc = GetDC(hwnd);
    screan.insert_i(DRAW_DBX, currentLineAttributes, pos, color);
    int t = step / 2;
    //int dx[15] = { 0,-t / 3,-t / 3 * 2,-t / 3,-t / 4,-t / 2,0, t/2,t/4,t/3,t/3*2,t/3};
    //int dy[15] = { -t,-t / 2,  t / 2, 0,t / 3 * 2,t / 3,t / 3 * 2,0,t / 2 - t / 2,-t };
    int dx[15] = { 0,-t / 6,-t / 2,t / 2,t / 6 };
    int dy[15] = { -t,0,t / 2,t / 2,0 };
    for (int i = 0; i < 5; i++) {
        screan.insert_p(x + dx[i], y + dy[i]);
    }

    for (size_t i = 0; i < screan.temp.size() - 1; i++) {
        DrawLine(hdc, screan.temp[i].x, screan.temp[i].y, screan.temp[i + 1].x, screan.temp[i + 1].y);
    }
    DrawLine(hdc, screan.temp[screan.temp.size() - 1].x, screan.temp[screan.temp.size() - 1].y, screan.temp[0].x, screan.temp[0].y);
    screan.insert_g();
    ReleaseDC(hwnd, hdc);
    return screan.sum;
}
void draw_map(HWND hwnd) {
    int px, py, dx, dy;
    px = step * 5;
    py = step * 8;
    color = 0;
    int f = 0;
    dx = step * 13; dy = 0;
    int pos = 76;
    int sx = step * 5, sy = step * 1;
    for (int i = 0; i < 4; i++) {
        int t = 76;
        board[t + i * 4 + 0].x = sx; board[t + i * 4 + 0].y = sy; board[t + i * 4 + 0].col = color;
        board[t + i * 4 + 1].x = sx + step; board[t + i * 4 + 1].y = sy; board[t + i * 4 + 1].col = color;
        board[t + i * 4 + 2].x = sx + step; board[t + i * 4 + 2].y = sy + step; board[t + i * 4 + 2].col = color;
        board[t + i * 4 + 3].x = sx; board[t + i * 4 + 3].y = sy + step; board[t + i * 4 + 3].col = color;
        for (int j = 0; j < 4; j++) {
            board[t + i * 4 + j].plant = draw_plant(hwnd, board[t + i * 4 + j].x, board[t + i * 4 + j].y, t + i * 4 + j);
        }
        sx += dx; sy += dy;
        int tt = dx;
        dx = -dy; dy = tt;
        change_color();
    }
    dx = 0;
    dy = -step;
    pos = 0;
    for (int i = 0; i < 4; i++) {
        sx = px; sy = py;
        int t = 52;
        int ddx = -dy, ddy = dx;
        for (int j = 0; j < 6; j++) {
            sx += ddx; sy += ddy;
            board[t + i * 6 + j].x = sx;
            board[t + i * 6 + j].y = sy;
            board[t + i * 6 + j].col = color;
        }



        board[pos].x = px;
        board[pos].y = py;
        board[pos].col = color;

        board[pos].lab = 52 + i * 6;
        change_color();
        pos++;
        for (int j = 0; j < 3; j++) {
            px += dx; py += dy;
            board[pos].x = px;
            board[pos].y = py;
            board[pos].col = color;
            pos++;
            change_color();
        }
        int d = dx;
        dx = -dy; dy = d;
        for (int j = 0; j < 3; j++) {
            px += dx; py += dy;
            board[pos].x = px;
            board[pos].y = py;
            board[pos].col = color;
            pos++;
            change_color();
        }
        px += dx; py += dy;
        d = dy;
        dy = -dx; dx = d;
        for (int j = 0; j < 4; j++) {
            px += dx; py += dy;
            board[pos].x = px;
            board[pos].y = py;
            board[pos].col = color;
            if (pos % 13 == 7)
                board[pos].lab = (pos + 12) % 52;

            pos++;
            change_color();
        }
        d = dx;
        dx = -dy; dy = d;
        px += dx; py += dy;
        for (int j = 0; j < 2; j++) {
            board[pos].x = px;
            board[pos].y = py;
            board[pos].col = color;
            pos++;
            change_color();
            px += dx; py += dy;
        }
    }

}
std::stack<std::pair<int, int>> s;
bool ISbroad(int x, int y) {
    if (x < width && x > 0) {
        if (y < height && y > 0) {
            return 1;
        }
    }
    return 0;
}
void FloodFill4(int x, int y, int oldColor, int newColor)
{
    s.push(std::make_pair(x, y));
    while (!s.empty())
    {
        std::pair<int, int> p = s.top();
        s.pop();

        if (window[p.first][p.second] == oldColor)
        {
            //SetPixel(hdc, p.first, p.second, newColor);
            window[p.first][p.second] = newColor;
            if (ISbroad(p.first, p.second)) {
                if (window[p.first][p.second + 1] == oldColor) //检测上点
                    s.push(std::make_pair(p.first, p.second + 1));
                if (window[p.first][p.second - 1] == oldColor) //检测下点
                    s.push(std::make_pair(p.first, p.second - 1));
                if (window[p.first - 1][p.second] == oldColor) //检测左点
                    s.push(std::make_pair(p.first - 1, p.second));
                if (window[p.first + 1][p.second] == oldColor) //检测右点
                    s.push(std::make_pair(p.first + 1, p.second));
            }

        }
    }
}
HDC background;
HDC memDC = CreateCompatibleDC(background);
void draw_window(HWND hwnd) {
    HDC hdc = GetDC(hwnd);
    HDC memDC = CreateCompatibleDC(hdc);
    HBITMAP hBitmap = CreateCompatibleBitmap(hdc, width, height);
    SelectObject(memDC, hBitmap);
    // 绘制图像到缓冲区
    for (int x = 0; x < width; x++) {
        for (int y = 0; y < height; y++) {
            SetPixel(memDC, x, y, window[x][y]);
        }
    }
    // 将缓冲区中的图像绘制到窗口
    BitBlt(hdc, 0, 0, width, height, memDC, 0, 0, SRCCOPY);
    // 清理资源
    DeleteObject(hBitmap);
    DeleteDC(memDC);
}
void draw_graph(HWND hwnd, int i)
{
    int drawtype = screan.gragh_inf[i].lab;
    if (drawtype == DRAW_LINE_Bre) {
        HDC hdc = GetDC(hwnd);
        DrawLine(hdc, screan.point[i][0].x, screan.point[i][0].y, screan.point[i][1].x, screan.point[i][1].y);
        ReleaseDC(hwnd, hdc);
    }
    else if (drawtype == DRAW_LINE_Mid) {
        HDC hdc = GetDC(hwnd);
        DrawLine_Mid(hdc, screan.point[i][0].x, screan.point[i][0].y, screan.point[i][1].x, screan.point[i][1].y);
        ReleaseDC(hwnd, hdc);
    }
    else if (drawtype == DRAW_CIRCLE) {
        HDC hdc = GetDC(hwnd);
        MidPointCircle(hdc, screan.point[i][0].x, screan.point[i][0].y, screan.point[i][1].x, screan.point[i][1].y);
        ReleaseDC(hwnd, hdc);
    }
    else if (drawtype == DRAW_Are) {
        double Angle1 = CalculateAngle(screan.point[i][0].x, screan.point[i][0].y, screan.point[i][1].x, screan.point[i][1].y);
        double Angle2 = CalculateAngle(screan.point[i][1].x, screan.point[i][1].y, screan.point[i][2].x, screan.point[i][2].y);
        double r = CalculateR(screan.point[i][0].x, screan.point[i][0].y, screan.point[i][2].x, screan.point[i][2].y);
        HDC hdc = GetDC(hwnd);
        DrawArc(hdc, screan.point[i][0].x, screan.point[i][0].y, r, Angle1, Angle2, color);
        ReleaseDC(hwnd, hdc);
    }
    else if (drawtype == DRAW_DBX) {
        HDC hdc = GetDC(hwnd);
        vector<p>temp = screan.point[i];
        for (size_t j = 0; j < temp.size() - 1; j++) {
            DrawLine(hdc, temp[j].x, temp[j].y, temp[j + 1].x, temp[j + 1].y);
        }
        DrawLine(hdc, temp[temp.size() - 1].x, temp[temp.size() - 1].y, temp[0].x, temp[0].y);
        ReleaseDC(hwnd, hdc);
    }
    else if (drawtype == DRAW_Beize) {
        HDC hdc = GetDC(hwnd);
        vector<p>temp = screan.point[i];
        for (size_t j = 0; j < temp.size() - 1; j++) {
            DrawLine(hdc, temp[j].x, temp[j].y, temp[j + 1].x, temp[j + 1].y);
        }
        DrawLine(hdc, temp[temp.size() - 1].x, temp[temp.size() - 1].y, temp[0].x, temp[0].y);
        ReleaseDC(hwnd, hdc);
    }
}
void clear(HWND hwnd) {
    int temp_color = color;
    color = 255 * 256 * 256 + 255 * 256 + 255;
    for (size_t j = 0; j < screan.point.size(); j++) {
        if (screan.gragh_inf[j].live == 1) {
            //draw_graph(hwnd, j);
            screan.gragh_inf[j].live = 0;
        }
    }
    for (size_t i = 0; i < 1201; i++) {
        for (size_t j = 0; j < 901; j++) {
            window[i][j] = white;
        }
    }
    draw_window(hwnd);
    color = temp_color;
}
p deCasteljau(const std::vector<p>& points, float t) {
    // de Casteljau算法的实现
    std::vector<p> tempPoints = points;

    while (tempPoints.size() > 1) {
        std::vector<p> newPoints;

        for (int i = 0; i < static_cast<int>(tempPoints.size()) - 1; ++i) {
            float x = (1.0f - t) * tempPoints[i].x + t * tempPoints[i + 1].x;
            float y = (1.0f - t) * tempPoints[i].y + t * tempPoints[i + 1].y;
            p temp;
            temp.x = x; temp.y = y;
            newPoints.push_back(temp);
        }

        tempPoints = newPoints;
    }

    return tempPoints[0];
}
void DrawBeize(vector<p> line) {
    // 绘制Bezier曲线的代码
    if (line.size() < 2) {
        return;
    }
    p pre;
    for (float t = 0.0f; t < 1.0f; t += 0.0001f) {
        p point = deCasteljau(line, t);
        window[point.x][point.y] = color;
    }
}
void ChangeBezier(int shapeNum, int movex, int movey) {
    int tempcolor = color;
    color = white;
    DrawBeize(screan.point[shapeNum]);
    screan.point[shapeNum][BZlabel].x += movex;
    screan.point[shapeNum][BZlabel].y += movey;
    color = tempcolor;
    DrawBeize(screan.point[shapeNum]);
}
void draw_chess() {

}
int Find_Nearest_Chess(int x, int y, int thischessNum = -1) {
    double minDistance = INT_MAX;
    int nearestShapeIndex = -1;
    for (int i = 0; i < screan.point.size(); i++) {
        if (screan.gragh_inf[i].lab == DRAW_DBX && i != thischessNum) {
            for (int j = 0; j < screan.point[i].size(); j++) {
                double distance = sqrt(pow(screan.point[i][j].x - x, 2) + pow(screan.point[i][j].y - y, 2));
                if (distance < minDistance) {
                    minDistance = distance;
                    nearestShapeIndex = i;
                }
            }
        }
    }
    return nearestShapeIndex;
}

void ShowDice(int DiceNumber) {
    if (DiceNumber = 1) {
    }
    else if (DiceNumber = 2) {

    }
    else if (DiceNumber = 3) {

    }
    else if (DiceNumber = 4) {

    }
    else if (DiceNumber = 5) {

    }
    else if (DiceNumber = 6) {

    }
}
void Moveto(HWND hwnd, int currentSelect, int pos, int tar) {
    if (board[tar].plant != 0) {
        int col = screan.gragh_inf[board[tar].plant].color;
        int p = 0;
        while (col > 0) {
            p += 1;
            col = col / 256;
        }
        p = p * 4 + 76;
        while (board[p].plant != 0) {
            p++;
        }
        Moveto(hwnd, board[tar].plant, tar, p);
    }
    board[pos].plant = 0;
    screan.gragh_inf[currentSelect].pos = tar;
    color = white;
    int dx = board[pos].x - board[tar].x;
    int dy = board[pos].y - board[tar].y;
    draw_graph(hwnd, currentSelect);
    for (int i = 0; i < screan.point[currentSelect].size(); i++) {
        screan.point[currentSelect][i].x -= dx;
        screan.point[currentSelect][i].y -= dy;
    }
    color = screan.gragh_inf[currentSelect].color;
    draw_graph(hwnd, currentSelect);
    draw_window(hwnd);
    board[tar].plant = currentSelect;
}
void GoHome(int chessnum) {

}
void ChessAchieve(int chessnum) {
    //删除棋子，颜色方计数器++
}
// 处理键盘按键事件
void HandleKeyPress(HWND hwnd, WPARAM wParam) {
    HDC hdc = GetDC(hwnd);
    switch (wParam) {
    case VK_F1:
        // 用户按下 'f1' 键的操作 bre
        currentDrawType = DRAW_LINE_Bre;
        break;

    case VK_SPACE:
        // 用户按下 ' ' 键的操作 clear
        clear(hwnd);
        break;

    case VK_F2:
        // 用户按下 'f2' 键的操作 zhongdian
        currentDrawType = DRAW_LINE_Mid;
        break;

    case VK_F3:
        // 用户按下 'F3' 键的操作  yuan
        currentDrawType = DRAW_CIRCLE;
        break;

    case VK_F4:
        // 用户按下 'F4' 键的操作  hu
        currentDrawType = DRAW_Are;
        break;

    case VK_F5:
        // 用户按下 'F5' 键的操作  多边形
        currentDrawType = DRAW_DBX;
        break;

    case '1':
        // 用户按下 '1' 键的操作，切换线型为实线
        currentLineAttributes.style = LINE_SOLID;
        break;

    case '2':
        // 用户按下 '2' 键的操作，切换线型为虚线
        currentLineAttributes.style = LINE_DASH;
        break;

    case '3':
        // 用户按下 '3' 键的操作，切换线型为点线
        currentLineAttributes.style = LINE_DOT;
        break;

    case 'C':
        // 用户按下 'x' 键的操作，change color
        change_color();
        break;


    case 'S':
        // 用户按下 'S' 键的操作，choose graph
        if (currentDrawType != CHOOSE) {
            currentDrawType = CHOOSE;
            f_select = 1;
        }
        else {
            f_select = 1 - f_select;
            if (f_select == 0) {
                screan.currentSelect = -1;
            }
        }
        break;

    case 'V':
        // 用户按下 'V' 键的操作，choose graph
        currentDrawType = gamedice;
        g_select = 1;
        st = Dice(hwnd);
        draw_window(hwnd);
        break;
    case 'Q':
        g_select = 1 - g_select;
        if (g_select == 0) {
            screan.currentSelect = -1;
        }
        break;
    case 'A':
        if (f_select == 1)
            currentDrawType = ADD;
        break;
    case 'D':
        if (f_select == 1)
            currentDrawType = DISADD;
        break;
    case 'M':
        if (f_select == 1)
            currentDrawType = MOVE;
        break;
    case 'R':
        if (f_select == 1)
            currentDrawType = ROLL;
        break;
    case 'B':
        currentDrawType = DRAW_Beize;
        break;
    case 'G':
        //游戏
        currentDrawType = game;
        draw_map(hwnd);
        for (int i = 0; i < 92; i++) {
            color = board[i].col;
            Draw_Circle(hdc, board[i].x, board[i].y, step * 0.4, color);
        }

        draw_window(hwnd);
        break;

    case 'L':
        if (IfDiceUsed == false) {
            std::srand(static_cast<unsigned int>(std::time(nullptr)));
            DiceValue = (std::rand() % 6) + 1;
            IfDiceUsed = true;
            ShowDice(DiceValue);
        }
        break;

    case 'F':
        // 用户按下 'F' 键的操作，fill
        currentDrawType = FILL;
        break;

    case 'T':
        // 用户按下 'T' 键的操作，TRANSFORM BEIZE
        currentDrawType = CHANGE_BZ;
        break;
    case 'X':
        // 用户按下 'X' 键的操作，cut_MID
        currentDrawType = CUT_LINE_MID;
        break;
    case 'Z':
        // 用户按下 'X' 键的操作，cut_CS
        currentDrawType = CUT_LINE_CS;
        break;
    case VK_UP:
        // 用户按下上箭头键，增加线宽
        if (currentLineAttributes.width < 10) {
            currentLineAttributes.width++;
        }
        break;

    case VK_DOWN:
        // 用户按下下箭头键，减小线宽
        if (currentLineAttributes.width > 1) {
            currentLineAttributes.width--;
        }
        break;
    }
    ReleaseDC(hwnd, hdc);

}
LRESULT CALLBACK WindowProc(HWND hwnd, UINT uMsg, WPARAM wParam, LPARAM lParam) {
    static POINT startPoint = { -1, -1 };
    static POINT endPoint = { -1, -1 };

    switch (uMsg) {
    case WM_LBUTTONDOWN:
        startPoint.x = LOWORD(lParam);
        startPoint.y = HIWORD(lParam);
        endPoint.x = -1;
        endPoint.y = -1;
        return 0;

    case WM_LBUTTONUP:
        endPoint.x = LOWORD(lParam);
        endPoint.y = HIWORD(lParam);

        // 根据当前的绘制类型调用相应的绘制函数
        if (currentDrawType == DRAW_LINE_Bre) {
            if (startPoint.x != -1 && endPoint.x != -1) {
                screan.insert_i(DRAW_LINE_Bre, currentLineAttributes);
                screan.insert_p(startPoint.x, startPoint.y);
                screan.insert_p(endPoint.x, endPoint.y);
                screan.insert_g();
                HDC hdc = GetDC(hwnd);
                DrawLine(hdc, startPoint.x, startPoint.y, endPoint.x, endPoint.y);
                ReleaseDC(hwnd, hdc);
            }
        }
        else if (currentDrawType == CUT_LINE_MID) {
            int xmin, xmax, ymin, ymax;
            if (startPoint.x < endPoint.x) {
                xmin = startPoint.x;
                xmax = endPoint.x;
            }
            else {
                xmax = startPoint.x;
                xmin = endPoint.x;
            }
            if (startPoint.y < endPoint.y) {
                ymin = startPoint.y;
                ymax = endPoint.y;
            }
            else {
                ymax = startPoint.y;
                ymin = endPoint.y;
            }
            HDC hdc = GetDC(hwnd);
            ClipMidPointSubdivision(hdc, xmin, ymin, xmax, ymax);
            ReleaseDC(hwnd, hdc);
        }
        else if (currentDrawType == CUT_LINE_CS) {
            int xmin, xmax, ymin, ymax;
            if (startPoint.x < endPoint.x) {
                xmin = startPoint.x;
                xmax = endPoint.x;
            }
            else {
                xmax = startPoint.x;
                xmin = endPoint.x;
            }
            if (startPoint.y < endPoint.y) {
                ymin = startPoint.y;
                ymax = endPoint.y;
            }
            else {
                ymax = startPoint.y;
                ymin = endPoint.y;
            }
            HDC hdc = GetDC(hwnd);
            CohenSutherlandClipAll(hdc, xmin, ymin, xmax, ymax);
            ReleaseDC(hwnd, hdc);
        }
        else if (currentDrawType == CHOOSE) {
            if (f_select == 1) {
                screan.currentSelect = findNearestShape(startPoint.x, startPoint.y);
            }
        }
        else if (currentDrawType == gamedice) {
            if (g_select == 1) {
                screan.currentSelect = findNearestShape(startPoint.x, startPoint.y);
            }

            //int st = Dice(hwnd);
            int pos = screan.gragh_inf[screan.currentSelect].pos;
            if (pos >= 76) {
                int xx = (pos - 76) / 4;
                xx = xx * 13 + 3;
                Moveto(hwnd, screan.currentSelect, pos, xx);
            }
            else if (pos >= 52) {
                int x = pos - 52;
                int y = x % 6;
                x = x - y;
                if (y + st > 5) {
                    y = 10 - y - st;
                }
                else y += st;
                int t = 52 + x + y;
                Moveto(hwnd, screan.currentSelect, pos, t);

            }
            else {
                int t = screan.gragh_inf[screan.currentSelect].pos + st;
                if (board[t].lab != -1 && board[t].col == screan.gragh_inf[screan.currentSelect].color)//横穿
                {
                    t = board[t].lab;
                    Moveto(hwnd, screan.currentSelect, pos, t);

                }
                else {
                    t = t % 52;
                    if (pos / 13 != t / 13 && board[t - t % 13].col == screan.gragh_inf[screan.currentSelect].color) {
                        int p = 52 + t / 13 * 6;
                        if (t % 13 > 0) {
                            Moveto(hwnd, screan.currentSelect, pos, p + t % 13 - 1);
                        }
                        else {
                            Moveto(hwnd, screan.currentSelect, pos, t);
                        }
                    }
                    else if (pos % 13 == 0 && board[t - t % 13].col == screan.gragh_inf[screan.currentSelect].color) {
                        if (board[pos].col == screan.gragh_inf[screan.currentSelect].color) {
                            int p = 52 + t / 13 * 6;
                            Moveto(hwnd, screan.currentSelect, pos, p + t % 13 - 1);
                        }
                    }
                    else {
                        if (board[t].col == screan.gragh_inf[screan.currentSelect].color) {
                            t += 4;
                        }
                        t = t % 52;
                        Moveto(hwnd, screan.currentSelect, pos, t);
                    }
                }
            }
        }
        else if (currentDrawType == MOVE) {
            if (startPoint.x != -1 && endPoint.x != -1) {
                int temp_c = color;
                color = white;
                int dx = startPoint.x - endPoint.x;
                int dy = startPoint.y - endPoint.y;

                draw_graph(hwnd, screan.currentSelect);
                for (int i = 0; i < screan.point[screan.currentSelect].size(); i++) {
                    screan.point[screan.currentSelect][i].x -= dx;
                    screan.point[screan.currentSelect][i].y -= dy;
                }
                color = temp_c;
                draw_graph(hwnd, screan.currentSelect);
                draw_window(hwnd);
            }
        }
        else if (currentDrawType == ADD) {
            if (startPoint.x != -1 && endPoint.x != -1) {
                int temp_c = color;
                color = white;
                int dx = startPoint.x - endPoint.x;
                int dy = startPoint.y - endPoint.y;

                draw_graph(hwnd, screan.currentSelect);
                for (int i = 0; i < screan.point[screan.currentSelect].size(); i++) {
                    screan.point[screan.currentSelect][i].x = startPoint.x + (screan.point[screan.currentSelect][i].x - startPoint.x) * 2;
                    screan.point[screan.currentSelect][i].y = startPoint.y + (screan.point[screan.currentSelect][i].y - startPoint.y) * 2;
                }
                color = temp_c;
                draw_graph(hwnd, screan.currentSelect);
                draw_window(hwnd);
            }
        }
        else if (currentDrawType == DISADD) {
            if (startPoint.x != -1 && endPoint.x != -1) {
                int temp_c = color;
                color = white;
                int dx = startPoint.x - endPoint.x;
                int dy = startPoint.y - endPoint.y;

                draw_graph(hwnd, screan.currentSelect);
                for (int i = 0; i < screan.point[screan.currentSelect].size(); i++) {
                    screan.point[screan.currentSelect][i].x = startPoint.x + (screan.point[screan.currentSelect][i].x - startPoint.x) / 2;
                    screan.point[screan.currentSelect][i].y = startPoint.y + (screan.point[screan.currentSelect][i].y - startPoint.y) / 2;
                }
                color = temp_c;
                draw_graph(hwnd, screan.currentSelect);
                draw_window(hwnd);
            }
        }
        else if (currentDrawType == ROLL) {
            if (startPoint.x != -1 && endPoint.x != -1) {
                int temp_c = color;
                color = white;
                draw_graph(hwnd, screan.currentSelect);
                double r = CalculateR(startPoint.x, startPoint.y, endPoint.x, endPoint.y);
                double dx = startPoint.x - endPoint.x;
                double dy = startPoint.y - endPoint.y;
                double cosTheta = dx / r;
                double sinTheta = dy / r;
                for (int i = 0; i < screan.point[screan.currentSelect].size(); i++) {
                    int x = screan.point[screan.currentSelect][i].x - startPoint.x;
                    int y = screan.point[screan.currentSelect][i].y - startPoint.y;
                    // 应用旋转矩阵
                    int newX = static_cast<int>(x * cosTheta - y * sinTheta + 0.5);
                    int newY = static_cast<int>(x * sinTheta + y * cosTheta + 0.5);
                    // 更新多边形顶点
                    screan.point[screan.currentSelect][i].x = newX + startPoint.x;
                    screan.point[screan.currentSelect][i].y = newY + startPoint.y;
                }
                color = temp_c;
                draw_graph(hwnd, screan.currentSelect);
                draw_window(hwnd);
            }
        }
        else if (currentDrawType == FILL) {
            HDC hdc = GetDC(hwnd);
            int oldcolor = GetPixel(hdc, startPoint.x, startPoint.y);
            FloodFill4(startPoint.x, startPoint.y, oldcolor, color);
            draw_window(hwnd);
            ReleaseDC(hwnd, hdc);
        }
        else if (currentDrawType == DRAW_LINE_Mid) {
            if (startPoint.x != -1 && endPoint.x != -1) {
                screan.insert_i(DRAW_LINE_Mid, currentLineAttributes);
                screan.insert_p(startPoint.x, startPoint.y);
                screan.insert_p(endPoint.x, endPoint.y);
                screan.insert_g();
                HDC hdc = GetDC(hwnd);
                DrawLine_Mid(hdc, startPoint.x, startPoint.y, endPoint.x, endPoint.y);
                ReleaseDC(hwnd, hdc);
            }
        }
        else if (currentDrawType == DRAW_CIRCLE) {
            if (startPoint.x != -1 && endPoint.x != -1) {
                screan.insert_i(DRAW_CIRCLE, currentLineAttributes);
                screan.insert_p(startPoint.x, startPoint.y);
                screan.insert_p(endPoint.x, endPoint.y);
                screan.insert_g();
                HDC hdc = GetDC(hwnd);
                MidPointCircle(hdc, startPoint.x, startPoint.y, endPoint.x, endPoint.y);
                ReleaseDC(hwnd, hdc);
            }
        }
        else if (currentDrawType == DRAW_Are) {
            screan.insert_i(DRAW_Are, currentLineAttributes);
            if (Are_count < 2) {
                screan.insert_p(startPoint.x, startPoint.y);
                HDC hdc = GetDC(hwnd);
                DrawPoint(hdc, endPoint.x, endPoint.y, color);
                ReleaseDC(hwnd, hdc);
                Are_count++;
            }
            else {
                screan.insert_p(startPoint.x, startPoint.y);
                double Angle1 = CalculateAngle(screan.temp[0].x, screan.temp[0].y, screan.temp[1].x, screan.temp[1].y);
                double Angle2 = CalculateAngle(screan.temp[0].x, screan.temp[0].y, screan.temp[2].x, screan.temp[2].y);
                double r = CalculateR(screan.temp[0].x, screan.temp[0].y, endPoint.x, endPoint.y);
                HDC hdc = GetDC(hwnd);
                DrawArc(hdc, screan.temp[0].x, screan.temp[0].y, r, Angle1, Angle2, color);
                screan.insert_g();
                ReleaseDC(hwnd, hdc);
                //重置
                Are_count = 0;
            }
        }
        else if (currentDrawType == DRAW_DBX) {
            if (jsq == 0) {
                screan.insert_i(DRAW_DBX, currentLineAttributes);
                jsq++;
            }
            if (jsq >= 1) {
                screan.insert_p(startPoint.x, startPoint.y);
                HDC hdc = GetDC(hwnd);
                DrawPoint(hdc, endPoint.x, endPoint.y, color);
                ReleaseDC(hwnd, hdc);
            }
        }
        else if (currentDrawType == DRAW_Beize) {
            if (bse == 0) {
                screan.insert_i(DRAW_DBX, currentLineAttributes);
                bse++;
            }
            if (bse >= 1) {
                screan.insert_p(startPoint.x, startPoint.y);
                HDC hdc = GetDC(hwnd);
                DrawPoint(hdc, endPoint.x, endPoint.y, color);
                ReleaseDC(hwnd, hdc);
            }
        }
        else if (currentDrawType == CHOOSE) {
            if (startPoint.x != -1 && endPoint.x != -1) {
                // diao yong hanshu
                screan.currentSelect = findNearestShape(startPoint.x, startPoint.y);
            }
        }
        else if (currentDrawType == CHANGE_BZ) {
            int movex = endPoint.x - startPoint.x;
            int movey = endPoint.y - startPoint.y;
            HDC hdc = GetDC(hwnd);

            ChangeBezier(screan.currentSelect, movex, movey);
            ReleaseDC(hwnd, hdc);
        }
        else if (currentDrawType == game) {
        }

        draw_window(hwnd);

        return 0;
    case WM_RBUTTONUP:
        if (currentDrawType == DRAW_DBX) {
            if (jsq == 1) {
                HDC hdc = GetDC(hwnd);
                for (size_t i = 0; i < screan.temp.size() - 1; i++) {
                    DrawLine(hdc, screan.temp[i].x, screan.temp[i].y, screan.temp[i + 1].x, screan.temp[i + 1].y);
                }
                DrawLine(hdc, screan.temp[screan.temp.size() - 1].x, screan.temp[screan.temp.size() - 1].y, screan.temp[0].x, screan.temp[0].y);
                ReleaseDC(hwnd, hdc);
                screan.insert_g();
                jsq = 0;
                draw_window(hwnd);
            }
        }
        else if (currentDrawType == DRAW_Beize) {
            if (bse == 1) {
                screan.insert_g();
                bse = 0;
                vector<p> temp = screan.point[screan.point.size() - 1];
                DrawBeize(temp);

                draw_window(hwnd);
            }
        }


        return 0;

    case WM_KEYDOWN:
        // 处理键盘按下事件
        HandleKeyPress(hwnd, wParam);

        return 0;

    case WM_PAINT:
    {
        PAINTSTRUCT ps;
        HDC hdc = BeginPaint(hwnd, &ps);

        // 如果起点和终点都有效，根据当前绘制类型执行绘制
        if (startPoint.x != -1 && endPoint.x != -1) {
            if (currentDrawType == DRAW_LINE) {
                DrawLine(hdc, startPoint.x, startPoint.y, endPoint.x, endPoint.y);
            }
            else if (currentDrawType == DRAW_RECTANGLE) {
                DrawLine_Mid(hdc, startPoint.x, startPoint.y, endPoint.x, endPoint.y);
            }
        }

        EndPaint(hwnd, &ps);
        return 0;
    }

    case WM_DESTROY:
        PostQuitMessage(0);
        return 0;
    }

    return DefWindowProc(hwnd, uMsg, wParam, lParam);
}
int WINAPI WinMain(HINSTANCE hInstance, HINSTANCE hPrevInstance, LPSTR lpCmdLine, int nCmdShow) {
    const wchar_t* className = L"DrawingAppClass";
    WNDCLASS wc = { 0 };
    wc.lpfnWndProc = WindowProc;
    wc.hInstance = hInstance;
    wc.lpszClassName = className;
    wc.hbrBackground = (HBRUSH)(COLOR_WINDOW + 1);
    RegisterClass(&wc);

    HWND hwnd = CreateWindow(className, L"Mouse Drawing Example", WS_OVERLAPPEDWINDOW, 100, 100, 1200, 900, NULL, NULL, hInstance, NULL);
    ShowWindow(hwnd, nCmdShow);

    clear(hwnd);

    MSG msg = { 0 };
    while (GetMessage(&msg, NULL, 0, 0)) {
        TranslateMessage(&msg);
        DispatchMessage(&msg);
    }

    return 0;
}
