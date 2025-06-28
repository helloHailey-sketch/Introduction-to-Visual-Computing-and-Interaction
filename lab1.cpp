#include <random>
#include <algorithm>
#include <cmath>
#include <iostream>
#include <ctime>
#include <cstdlib>
#include <spdlog/spdlog.h>
#include "Labs/1-Drawing2D/tasks.h"

using VCX::Labs::Common::ImageRGB;

namespace VCX::Labs::Drawing2D {
    /******************* 1.Image Dithering *****************/
    void DitheringThreshold(
        ImageRGB &       output,
        ImageRGB const & input) {
        for (std::size_t x = 0; x < input.GetSizeX(); ++x)
            for (std::size_t y = 0; y < input.GetSizeY(); ++y) {
                glm::vec3 color = input.At(x, y);
                output.At(x, y) = {
                    color.r > 0.5 ? 1 : 0,
                    color.g > 0.5 ? 1 : 0,
                    color.b > 0.5 ? 1 : 0,
                };
            }
    }

    void DitheringRandomUniform(
        ImageRGB &       output,
        ImageRGB const & input) {
        // your code here:

        for (std::size_t x = 0; x < input.GetSizeX(); ++x)
            for (std::size_t y = 0; y < input.GetSizeY(); ++y) {
                glm::vec3 color = input.At(x, y);
                
                    //生成[-0.5,0.5]随机数
                    //使用 std::random_device 作为随机数种子
                    std::random_device rd;
                    std::mt19937 rng(rd()); //使用随机设备生成的种子来初始化随机引擎

                    std::uniform_real_distribution<float> dist(-0.5f, 0.5f);

                    float noise = dist(rng);

                output.At(x, y) = {
                    color.r + noise > 0.5 ? 1 : 0,
                    color.g + noise > 0.5 ? 1 : 0,
                    color.b + noise > 0.5 ? 1 : 0,
                };
            }
    }

    void DitheringRandomBlueNoise(
        ImageRGB &       output,
        ImageRGB const & input,
        ImageRGB const & noise) {
        // your code here:
        for (std::size_t x = 0; x < input.GetSizeX(); ++x)
            for (std::size_t y = 0; y < input.GetSizeY(); ++y) {
                glm::vec3 color1 = input.At(x, y);
                glm::vec3 color2 = noise.At(x, y); //[0,1] -> [-0.5, 0.5]
                output.At(x, y) = {
                    color1.r + color2.r -0.5 > 0.5 ? 1 : 0,
                    color1.g + color2.g -0.5 > 0.5 ? 1 : 0,
                    color1.b + color2.b -0.5 > 0.5 ? 1 : 0,
                };
            }
    }

    void DitheringOrdered(
        ImageRGB &       output,
        ImageRGB const & input) {
        // your code here:
        int M[3][3] = {{6,8,4},{1,0,7},{5,2,3}};

        for (std::size_t x = 0; x < input.GetSizeX(); ++x){
            for (std::size_t y = 0; y < input.GetSizeY(); ++y) {
                glm::vec3 color = input.At(x, y);
                
                int r, g, b;
                //遍历output中的3*3
                for (int i = 0; i < 3; i++){
                    for (int j = 0; j < 3; j++){
                        //原图大于 Mij /9 则第 i 行第 j 列亮度为1，否则为 0
                        if (color.r * 9 >  M[i][j]){
                            r = 1; g = 1; b = 1;
                        }
                        else{ 
                            r = 0; g = 0; b = 0;
                        }
                        output.At(3*x+i, 3*y+j) = {r, g, b};
                    }
                }
                          
                };
            }
    }

    void DitheringErrorDiffuse(
        ImageRGB &       output,
        ImageRGB const & input) {
        // your code here:
        //生成和input一样的input2，在input2上操作，最后赋给output
        ImageRGB input2 = input;

        for (std::size_t y = 0; y < input.GetSizeY(); ++y){
            for (std::size_t x = 0; x < input.GetSizeX(); ++x) {
                glm::vec3 color_input = input.At(x, y);
                glm::vec3 color_input2 = input2.At(x, y);
                //先二极化
                output.At(x, y) = {
                    color_input2.r > 0.5 ? 1 : 0,
                    color_input2.g > 0.5 ? 1 : 0,
                    color_input2.b > 0.5 ? 1 : 0,
                };
                glm::vec3 color_output = output.At(x, y);
                glm::vec3 error =  color_input2 - color_output; //二值化前的亮度减去二值化后的亮度作为误差值
               
                if(x+1 < input.GetSizeX()){
                    glm::vec3 color = input2.At(x+1, y);
                    color += error * (float)7/(float)16;
                    input2.At(x+1, y) = {color.r, color.g, color.b};
                }
               
                if(x+1 < input.GetSizeX() && y+1 < input.GetSizeY()){
                    glm::vec3 color = input2.At(x+1, y+1);
                    color += error * (float)1/(float)16;
                    input2.At(x+1, y+1) = {color.r, color.g, color.b};
                }
                
                if(y+1 < input.GetSizeY()){
                    glm::vec3 color = input2.At(x, y+1);
                    color += error * (float)5/(float)16;
                    input2.At(x, y+1) = {color.r, color.g, color.b};
                }
                
                if(x >= 1 && y+1 < input.GetSizeY()){
                    glm::vec3 color = input2.At(x-1, y+1);
                    color += error * (float)3/(float)16;
                    input2.At(x-1, y+1) = {color.r, color.g, color.b};
                }

            }
        }
    }

    /******************* 2.Image Filtering *****************/
    void Blur(
        ImageRGB &       output,
        ImageRGB const & input) {
        // your code here:
        for (std::size_t x = 0; x < input.GetSizeX(); ++x){
            for (std::size_t y = 0; y < input.GetSizeY(); ++y) {
                
                glm::vec3 color_sum = {0,0,0};
                int count = 0; //计数：多少个格子求平均
                //遍历对应原图的3*3像素区
                for(int i = -1; i < 2; i++){
                    for(int j = -1; j < 2; j++){
                        if(x+i >= 0 && x+i <input.GetSizeX() && y+j >= 0 && y+j < input.GetSizeY()){
                            glm::vec3 color_pixel = input.At(x+i, y+j);
                            color_sum += color_pixel;
                            count ++;
                        }
                    }
                }
                float r = color_sum.r / (float)count;
                float g = color_sum.g / (float)count;
                float b = color_sum.b / (float)count;
                output.At(x,y)={r,g,b};

            }
        }
    }

    void Edge(
        ImageRGB &       output,
        ImageRGB const & input) {
        // your code here:
        //Gx绕中心选择180度
        float filterX[3][3] = {{1,0,-1},{2,0,-2},{1,0,-1}}; //为了与vec3相乘用float
        float filterY[3][3] = {{-1,-2,-1},{0,0,0},{1,2,1}};

        for (std::size_t x = 0; x < input.GetSizeX(); ++x){
            for (std::size_t y = 0; y < input.GetSizeY(); ++y) {
                //边角
                if (x == 0 || y == 0 || x == input.GetSizeX() -1 || y == input.GetSizeY()-1){
                    output.At(x,y) = {0,0,0};
                }

                else{
                    glm::vec3 colorX = {0,0,0}; //filterX后的结果
                    glm::vec3 colorY = {0,0,0}; //filterY后的结果

                    for(int i = -1; i < 2; i++){
                        for(int j = -1; j < 2; j++){
                            glm::vec3 color = input.At(x+i, y+j);

                            colorX += color * filterX[i+1][j+1];
                            colorY += color * filterY[i+1][j+1];
                        }
                    }

                    glm::vec3 color_result = sqrt(colorX * colorX + colorY * colorY);
                    output.At(x,y) = color_result;
                }
            }
        }
    }

    /******************* 3. Image Inpainting *****************/
    void Inpainting(
        ImageRGB &         output,
        ImageRGB const &   inputBack,
        ImageRGB const &   inputFront,
        const glm::ivec2 & offset) { //offset 为飞机对应部分相对于背景图像顶点的偏移
        output             = inputBack;
        std::size_t width  = inputFront.GetSizeX();
        std::size_t height = inputFront.GetSizeY();
        glm::vec3 * g      = new glm::vec3[width * height]; //编辑量记作 g
        memset(g, 0, sizeof(glm::vec3) * width * height);
        // set boundary condition 在代码中填写边界处的结果
        for (std::size_t y = 0; y < height; ++y) { //遍历每一行，设置左边界和右边界的初始值
            // set boundary for (0, y), your code: g[y * width] = ?
            // set boundary for (width - 1, y), your code: g[y * width + width - 1] = ?
            //g = inputBack - inputFront
            //!!y从0开始，x要-1
            g[y * width] = inputBack.At(offset.x, offset.y + y) - inputFront.At(0,y);
            g[y * width + width - 1] = inputBack.At(offset.x + width - 1, offset.y + y) - inputFront.At(width - 1, y);

        }
        for (std::size_t x = 0; x < width; ++x) { //遍历每一列，设置上边界和下边界
            // set boundary for (x, 0), your code: g[x] = ?
            // set boundary for (x, height - 1), your code: g[(height - 1) * width + x] = ?
            //x从0开始不用-1， y要
            g[x] = inputBack.At(offset.x + x, offset.y) - inputFront.At(x, 0);
            g[(height - 1) * width + x] = inputBack.At(offset.x + x, offset.y + height - 1) - inputFront.At(x, height - 1);
        }

        // Jacobi iteration, solve Ag = b
        for (int iter = 0; iter < 8000; ++iter) {
            for (std::size_t y = 1; y < height - 1; ++y)
                for (std::size_t x = 1; x < width - 1; ++x) {
                    g[y * width + x] = (g[(y - 1) * width + x] + g[(y + 1) * width + x] + g[y * width + x - 1] + g[y * width + x + 1]);
                    g[y * width + x] = g[y * width + x] * glm::vec3(0.25);
                }
        }

        for (std::size_t y = 0; y < inputFront.GetSizeY(); ++y)
            for (std::size_t x = 0; x < inputFront.GetSizeX(); ++x) {
                glm::vec3 color = g[y * width + x] + inputFront.At(x, y);
                output.At(x + offset.x, y + offset.y) = color;
            }
        delete[] g;
    }

    /******************* 4. Line Drawing *****************/
    void DrawLine(
        ImageRGB &       canvas,
        glm::vec3 const  color,
        glm::ivec2 const p0,
        glm::ivec2 const p1) {
        // your code here:
        if (p0.x == p1.x){
            //垂直或两点重合
            for(size_t t = std::min(p0.y, p1.y); t <= std::max(p0.y, p1.y); t++){
                canvas.At((size_t) p0.x, t) = color;
            }
        }
        else{
           float slope = (float(p1.y - p0.y)/(p1.x - p0.x));
           int x0 = p0.x, y0 = p0.y, x1 = p1.x, y1 = p1.y;
           if (p1.y < p0.y){
            //p1在p0下方：对调p0和p1
            x0 = p1.x; x1 = p0.x; y0 = p1.y; y1 = p0.y;
           } 
           if(slope < 1 && slope >= 0){
            int x, y = y0;
                int dx = 2 * ( x1 - x0 ), dy = 2 * (y1 - y0);
                int dydx = dy - dx, F = dy - dx/2;
                for(x=x0; x<=x1; x++){
                    canvas.At(x,y) = color;
                    if (F < 0) F += dy;
                    else {y++; F += dydx;} 
                }
           }
           else if (slope >= 1 ){
            //对调x,y
            int y, x = x0;
            int dx = 2 * ( x1 - x0 ), dy = 2 * (y1 - y0);
            int dydx = dx - dy, F = dx - dy/2;
            for(y=y0; y<=y1; y++){
                canvas.At(x,y) = color;
                if (F < 0) F += dx;
                else {x++; F += dydx;}
                }
            }
            else if ( slope < 0 && slope >= -1){
                //与标准情况比x变成-x
                int x, y = y0;
                int dx = 2 * ( x0 - x1 ), dy = 2 * (y1 - y0);
                int dydx =  dy - dx, F = dy - dx/2;
                for(x=x0; x>=x1; x--){
                    canvas.At(x,y) = color;
                    if (F < 0) F += dy;
                    else {y++; F += dydx;}
                }
            }
            else {
                //slope < -1: x变成-x且对调x，y
                int y, x = x0;
                int dx = 2 * ( x0 - x1 ), dy = 2 * (y1 - y0);
                int dydx = dx - dy, F = dx - dy/2;
                for(y=y0; y<=y1; y++){
                    canvas.At(x,y) = color;
                    if (F < 0) F += dx;
                    else {x--; F += dydx;}
                }
            }
        }  
    }

    /******************* 5. Triangle Drawing *****************/
    void DrawTriangleFilled(
        ImageRGB &       canvas,
        glm::vec3 const  color,
        glm::ivec2 const p0,
        glm::ivec2 const p1,
        glm::ivec2 const p2) {
        // your code here:
        //给3个点按照y从小到大排序 0<=1<=2
        int x0 = p0.x, x1 = p1.x, x2 = p2.x, y0 = p0.y, y1 = p1.y, y2 = p2.y;
        if (y0 > y1) {std::swap(x0, x1); std::swap(y0, y1);}
        if (y0 > y2) {std::swap(x0, x2); std::swap(y0, y2);}
        if (y1 > y2) {std::swap(x1, x2); std::swap(y1, y2);}
        canvas.At(x0,y0) = color;
        canvas.At(x1,y1) = color;
        canvas.At(x2,y2) = color;


        //三角形没有水平的边
        if(y1 != y0 && y0 != y2 ){ 
        //判断p1在p0p2左侧还是右侧
        float x_M = (float)(x2 - x0)/(y2 - y0)* (y1-y0) + x0; //p0p2上(x_M,y1)
        //画下半部
            float x_L = x0, x_R = x0; //initialize 两端点
            float dx_L = ((float)(x1 - x0)/(y1 - y0)); //根据斜率求y增加1时x的变化,默认p1在p0p2左侧
            float dx_R = ((float)(x2 - x0)/(y2 - y0));
            if (x1 > x_M) std::swap(dx_L, dx_R); //p1在右侧
            for(int y = y0; y <= y1; y++){
                for(int x = x_L; x <= x_R; x++) canvas.At(x,y) = color;
                x_L += dx_L;
                x_R += dx_R;
            }
            x_L -= dx_L;
            x_R -= dx_R;
            
        //画上半部
        //此时x_L为中间线左端点，x_R为右端点
        //以防x_L因为累加产生误差：重新设定
            dx_L = ((float)(x2 - x1)/(y2 - y1)); //根据斜率求y增加1时x的变化，默认p1在p0p2左侧
            dx_R = ((float)(x2 - x0)/(y2 - y0));
            x_L = x1; x_R = x_M;
 
            if (x_M < x1) {std::swap(dx_L, dx_R);std::swap(x_L, x_R);} //若p1在p0p2右侧
            for(int y = y1; y <= y2; y++){
                for(int x = x_L; x <= x_R; x++) canvas.At(x,y) = color;
                x_L += dx_L; 
                x_R += dx_R;
            } 
        }
        
        else if(y1 == y2 && y2 != y0){
                //上底平：只求下半
                float x_L = x0, x_R = x0; //initialize 两端点
                float dx_L = ((float)(x1 - x0)/(y1 - y0)); //根据斜率求y增加1时x的变化
                float dx_R = ((float)(x2 - x0)/(y2 - y0));
                if (x1 > x2) std::swap(dx_L, dx_R);
                for(int y = y0; y <= y1; y++){
                    for(int x = x_L; x <= x_R; x++) canvas.At(x,y) = color;
                    x_L += dx_L;
                    x_R += dx_R;
                }
            }

        else if(y2 != y0){//下底平：只求上半
        std::cout << "下底平" << std::endl;
        //判断p1在p0p2左侧还是右侧
        //float x_M = (float)(x2 - x0)/(y2 - y0)* (y1-y0) + x0; //p0p2上y坐标为y1的点
            float x_L = x0, x_R = x1; //initialize 两端点
            if (x0 > x1) std::swap(x_L, x_R);
            float dx_L = ((float)(x2 - x1)/(y2 - y1)); //根据斜率求y增加1时x的变化，默认p1在左侧
            float dx_R = ((float)(x2 - x0)/(y2 - y0));
            if (x0 < x1) {std::swap(dx_L, dx_R); } //若p1在p0p2右侧
            for(int y = y1; y <= y2; y++){
                for(int x = x_L; x <= x_R; x++) canvas.At(x,y) = color;
                x_L += dx_L;
                x_R += dx_R;
            }
            }
            else{
                //三点共横线
                int c1 = std::min(x0,x1);
                int x_L = std::min(c1,x2);
                int c2 = std::max(x0,x1);
                int x_R = std::max(c2,x2);
                for (int x = x_L; x <= x_R; x++){
                    canvas.At(x, y0) = color;
                }
            }
        }


    

    /******************* 6. Image Supersampling *****************/
    void Supersample(
        ImageRGB &       output,
        ImageRGB const & input,
        int              rate) {
        // your code here:
        int input_X = input.GetSizeX(), input_Y = input.GetSizeY();
        int output_X = output.GetSizeX(), output_Y = output.GetSizeY();
        float scale = (float)input_X / output_X;

        //方便计算坐标+1
        for (std::size_t x_out = 1; x_out <= output_X; ++x_out) {
            for (std::size_t y_out = 1; y_out <= output_Y; ++y_out) {
                
                float r = 0, g = 0, b = 0;
                //rate*rate像素区中心，起始点（所有坐标+1）
                float mid_X = scale * (2 * x_out - 1 ) / 2.0f;
                float mid_Y = scale * (2 * y_out - 1 ) /2.0f;
                float start_X = mid_X - rate/2.0f;
                float start_Y = mid_Y - rate/2.0f;
        

                //遍历rate*rate像素区
                for(float x = start_X; x < (start_X + rate); x++){
                    for(float y = start_Y; y < (start_Y + rate); y++){
                        //确保不越界：
                        float x_rate = std::min(x, (float)input_X); 
                        if(x < 1) x_rate = 1;
                        x_rate--;
                        float y_rate = std::min(y, (float)input_Y); 
                        if(y < 1) y_rate = 1;
                        y_rate--;

                        //求input中(x_rate,y_rate)的颜色
                        //坐标如果不为整数：通过插值计算

                            //找到周围四个点的坐标
                            int x0 = std::floor(x_rate);
                            int x1 = std::min(x0 + 1, input_X - 1);   // 防止越界
                            int y0 = std::floor(y_rate);
                            int y1 = std::min(y0 + 1, input_Y - 1);  // 防止越界

                            //计算权重
                            float t_X = x_rate - x0;
                            float t_Y = y_rate - y0;

                            //周围四个点的rgb
                            glm::vec3 C11 = input.At(x0, y0);
                            glm::vec3 C21 = input.At(x1, y0);
                            glm::vec3 C12 = input.At(x0, y1);
                            glm::vec3 C22 = input.At(x1, y1);

                            //双线性插值
                            glm::vec3 top = C11 * (1 - t_X) + C21 * t_X;  // 在x方向上插值（上行）
                            glm::vec3 bottom = C12 * (1 - t_X) + C22 * t_X;  // 在x方向上插值（下行）
                            glm::vec3 result = top * (1 - t_Y) + bottom * t_Y;  // 在y方向上插值

 
                        r += result.r;
                        g += result.g;
                        b += result.b;
                    }
                }

                r /= (rate * rate); g /= (rate * rate); b /= (rate * rate);
                output.At((x_out - 1),(y_out - 1)) = {r, g, b};
                
                }
            }

    }


    /******************* 7. Bezier Curve *****************/
    // Note: Please finish the function [DrawLine] before trying this part.
    glm::vec2 BezierCurve (std::vector<glm::vec2> points, float t){
        if(points.size() == 1) return points[0];

        std::vector<glm::vec2> newPoints;
        for (int i = 0; i < points.size() - 1; ++i) {
        glm::vec2 p0 = points[i];
        glm::vec2 p1 = points[i + 1];
        glm::vec2 newPoint = (1 - t) * p0 + t * p1; // 线性插值
        newPoints.push_back(newPoint); //为了用pushback使用std::vector
        }

        return BezierCurve (newPoints, t);
        
    }
    glm::vec2 CalculateBezierPoint(
        std::span<glm::vec2> points,
        float const          t) {
        // your code here:
        std::vector<glm::vec2> Points(points.begin(), points.end());
        return BezierCurve (Points, t); //函数定义见上

        return glm::vec2 {0, 0};
    }
} // namespace VCX::Labs::Drawing2D