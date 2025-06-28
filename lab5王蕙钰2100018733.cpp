#include "Labs/5-Visualization/tasks.h"

#include <numbers>
#include <iostream>
#include <cmath>

using VCX::Labs::Common::ImageRGB;
namespace VCX::Labs::Visualization {

    struct CoordinateStates {
        // your code here
        std::vector<Car>data;
    };

    bool PaintParallelCoordinates(Common::ImageRGB & input, InteractProxy const & proxy, std::vector<Car> const & data, bool force) {
        // your code here
        // for example: 
        //   static CoordinateStates states(data);
        //   SetBackGround(input, glm::vec4(1));
        //   ...
        static CoordinateStates states(data);
        SetBackGround(input, glm::vec4(1));

        //7个数据的端点
        float x[7];
        for(int i = 0; i < 7; i++){
            x[i] = i * 0.9 / 6.0 + 0.05; //左右两侧空0.5
        }
        //DrawLine(Common::ImageRGB & canvas, glm::vec4 color, glm::vec2 from, glm::vec2 to, float width)
        //坐标轴
        for(int i = 0; i < 7; i++){
            DrawLine(input,glm::vec4(0,0,0,1), glm::vec2(x[i], 0.1), glm::vec2(x[i], 0.9), 0.05);
        }
        //DrawRect(Common::ImageRGB & canvas, glm::vec4 color, glm::vec2 leftTop, glm::vec2 size, float width)
        //DrawRect(input, glm::vec4(0,0,0,1), glm::vec2(0.1,0.1),glm::vec2(0.05,0.6), 0.005);
        //PrintText(Common::ImageRGB & canvas, glm::vec4 color, glm::vec2 pos, float lineHeight, std::string const & caption)
        std::string type[7] = { "cylinders", "displacement", "weight", "horsepower", "acceleration(0-60mph)", "mileage", "year" };
        std::string num1[7] = { "9", "494", "5493", "249", "27", "51", "84" };
        std::string num2[7] = { "2", "29", "1260", "27", "6", "5", "68" };
        for (int i = 0; i < 7; ++i) {
            PrintText(input, glm::vec4(0.0, 0.0, 0.0, 1), glm::vec2(x[i],0.05), 0.02, type[i]);
            PrintText(input, glm::vec4(0.0, 0.0, 0.0, 1), glm::vec2(x[i]-0.005, 0.08), 0.02, num1[i]);
            PrintText(input, glm::vec4(0.0, 0.0, 0.0, 1), glm::vec2(x[i]-0.005, 0.92), 0.02, num2[i]);
        }
        //渐变色
        int n = data.size();
        for(int i = 0; i < n; i++){
            DrawLine(input, glm::vec4(float(i)/float(n),0.6,0.6,1),glm::vec2(x[0], 0.1+0.8*(9-data[i].cylinders)/ (9 - 2)), glm::vec2(x[1], (494-data[i].displacement)/(494-29)*0.8+0.1),0.01);
            DrawLine(input, glm::vec4(float(i)/float(n),0.6,0.6,1),glm::vec2(x[1], (494-data[i].displacement)/(494-29)*0.8+0.1), glm::vec2(x[2], (5493-data[i].weight)/(5493-1260)*0.8+0.1),0.01);
            DrawLine(input, glm::vec4(float(i)/float(n),0.6,0.6,1),glm::vec2(x[2], (5493-data[i].weight)/(5493-1260)*0.8+0.1), glm::vec2(x[3], (249-data[i].horsepower)/(249-27)*0.8+0.1),0.01);
            DrawLine(input, glm::vec4(float(i)/float(n),0.6,0.6,1),glm::vec2(x[3], (249-data[i].horsepower)/(249-27)*0.8+0.1), glm::vec2(x[4], (27-data[i].acceleration)/(27-6)*0.8+0.1),0.01);
            DrawLine(input, glm::vec4(float(i)/float(n),0.6,0.6,1),glm::vec2(x[4], (27-data[i].acceleration)/(27-6)*0.8+0.1), glm::vec2(x[5], (51-data[i].mileage)/(51-5)*0.8+0.1),0.01);
            DrawLine(input, glm::vec4(float(i)/float(n),0.6,0.6,1),glm::vec2(x[5], (51-data[i].mileage)/(51-5)*0.8+0.1), glm::vec2(x[6], 0.8*(84-data[i].year)/(84-68)+0.1),0.01);
        }    
        return true;
    }

    void LIC(ImageRGB & output, Common::ImageRGB const & noise, VectorField2D const & field, int const & step) {
        // your code here
    int width = field.size.first;
    int height = field.size.second;
    //float weight = 0.0f;

    for(int y = 0; y < height; y++){
        for(int x = 0; x < width; x++ ){
            glm::vec2 current_position (x,y);
            glm::vec2 velocity = field.At(x,y);

            glm::vec3 accumulated_color (0,0,0);
            float accumulated_weight = 0.0f;

            //advect forwards and backwards
            for(int k = -step; k <= step; k++){
                glm::vec2 offset = float(k) * velocity;
                glm::vec2 new_position = current_position + offset;

                if(new_position.x >= 0 && new_position.x < width && new_position.y >= 0 && new_position.y < height){
                   float weight = std::pow(std::cos(0.46 * k), 2);
                    //(x,y)经过dt的坐标的noise * weight 加到(x,y)上
                    accumulated_color += noise.At(new_position.x, new_position.y) * weight;
                    accumulated_weight += weight; 
                }
            }//k iter end

            accumulated_color /= accumulated_weight;
            output.At(x,y) = accumulated_color;
        }//x iter end
    }//y iter end

        //field:速度 std::pair<std::uint32_t, std::uint32_t> size; std::vector<glm::vec2> vectors;
    /*    for(int i = 0; i < field.size.second; i++){
            for(int j = 0; j < field.size.first; j++){
                int x = j; 
                int y = i;
                glm::vec3 forward_sum (0.0f, 0.0f, 0.0f);
                float forward_total = 0;

                //Advect forwards
                for(int k = 0; k < step; k++){
                    //dx,dy:(x,y)点的速度的x，y分量
                    float dx = field.At(x,y).x;
                    float dy = field.At(x,y).y;

                    //dt_x,dt_y:(x,y)点的粒子穿越x/y网格边界所用的时间
                    float dt_x = 0;
                    float dt_y = 0;
                    //向x正方向走：走到下一个x所需要的时间
                    if(dx > 0) dt_x = (std::floor(x)+1-x)/dx;
                    //x负方向
                    else if(dx < 0) dt_x = (x-std::ceil(x)+1)/-dx;
                    //y正方向
                    if(dy > 0) dt_y = (std::floor(y)+1-y)/dy;
                    //y负方向
                    else if(dy < 0) dt_y = (y-std::ceil(y)+1)/-dy;
                    float dt;
                    if(dx == 0 && dy == 0) dt = 0;
                    else dt = std::min(dt_x, dt_y);

                    //确保粒子在经过dt后的坐标没越界
                    x = std::min(std::max(x+dx*dt,0.0f),field.size.first-1.0f);
                    y = std::min(std::max(y+dy*dt,0.0f),field.size.second-1.0f);

                    float weight = std::pow(std::cos(0.46 * k), 2);
                    //(x,y)经过dt的坐标的noise * weight 加到(x,y)上
                    forward_sum += noise.At(x, y) * weight;
                    forward_total += weight;
                }
                
                //Advect backwards
                x = j; 
                y = i;
                glm::vec3 backward_sum (0.0f, 0.0f, 0.0f); //noise*weight和
                float backward_total = 0;  //weight和
                //注意起始点k=1！！
                for(int k = 1; k < step; k++){
                    //dx,dy:(x,y)点的速度的x，y分量(正方向不变，因此都为负)
                    float dx = field.At(x,y).x * (-1);
                    float dy = field.At(x,y).y * (-1);

                    //dt_x,dt_y:(x,y)点的粒子穿越x/y网格边界所用的时间
                    float dt_x = 0;
                    float dt_y = 0;
                    //向x正方向走：走到下一个x所需要的时间
                    if(dx > 0) dt_x = (std::floor(x)+1-x)/dx;
                    //x负方向
                    else if(dx < 0) dt_x = (x-std::ceil(x)+1)/-dx;
                    //y正方向
                    if(dy > 0) dt_y = (std::floor(y)+1-y)/dy;
                    //y负方向
                    else if(dy < 0) dt_y = (y-std::ceil(y)+1)/-dy;
                    float dt;
                    if(dx == 0 && dy == 0) dt = 0;
                    else dt = std::min(dt_x, dt_y);

                    //确保粒子在经过dt后的坐标没越界
                    x = std::min(std::max(x+dx*dt,0.0f),field.size.first-1.0f);
                    y = std::min(std::max(y+dy*dt,0.0f),field.size.second-1.0f);

                    float weight = std::pow(std::cos(-0.46 * k), 2);
                    //(x,y)经过dt的坐标的noise * weight 加到(x,y)上
                    backward_sum += noise.At(x, y) * weight;
                    backward_total += weight;
                }//k结束

                output.At(j,i) = (forward_sum + backward_sum)/(forward_total + backward_total);

            }//j结束
        }//i结束
*/
    }//void结束
}; // namespace VCX::Labs::Visualization